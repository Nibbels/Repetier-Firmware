"""Shared linked AVR build and verification, including bounded recovery layout search.
No deployment authorization. Keep historical experiment callers and future standard
build caller on the same recovery-link and watchdog verification path.
"""
import json
from pathlib import Path
import re
import shutil
import subprocess

def archive_failed(archive, directory, mode, label):
    if archive:
        failed=archive/mode/label
        failed.mkdir(parents=True,exist_ok=True)
        for name in ("compile.txt","properties.txt","properties-with-linker.txt","avr6-recovery.x","build/firmware.map"):
            if (directory/name).is_file():
                shutil.copy2(directory/name,failed/Path(name).name)


def build_verified(baseline, recovery, timing, cli, mode_dir, board, definitions,
                   use_recovery, archive=None, mode="combined"):
    board_dir=mode_dir/board
    try:
        stats=baseline.build_target(mode_dir,cli,board,definitions,
                                    linker_script_factory=recovery.make_script if use_recovery else None)
    except subprocess.CalledProcessError:
        log=board_dir/"compile.txt"
        map_file=board_dir/"build/firmware.map"
        log_text=log.read_text(errors="replace") if log.exists() else ""
        print(log_text[-24000:],flush=True)
        archive_failed(archive, board_dir,mode,board+"-failed")
        if not use_recovery or not ("cannot move location counter backwards" in log_text or
                             "text section exceeds available space in board" in log_text):
            raise
        # A backwards-counter failure often leaves an incomplete map.
        # Link a disposable sizing probe with no fixed recovery hole.
        # Only its map and statistics are evidence, never its image.
        # If the ordinary partition inventory cannot fit, use a
        # finer balanced WPA partitioning. This is a bounded search
        # over linker configurations, not a bypass of linked audits.
        for partition_count in (None,8,16):
            label="layout-probe"+("-lto"+str(partition_count) if partition_count else "")
            probe_dir=mode_dir/label
            probe_dir.mkdir(exist_ok=True)
            probe_board=probe_dir/board
            try:
                probe_stats=baseline.build_target(probe_dir,cli,board,definitions,
                    linker_script_factory=lambda bin_dir,directory: recovery.make_script(bin_dir,directory,probe=True),
                    recovery_lto_partitions=partition_count)
            except subprocess.CalledProcessError:
                archive_failed(archive, probe_board,mode,board+"-"+label+"-failed")
                raise
            probe_map=probe_board/"build/firmware.map"
            if archive:
                dest=archive/mode/(board+"-"+label)
                dest.mkdir()
                for name in ("firmware.map","avr6-recovery.x","properties-with-linker.txt","compile.txt"):
                    source=probe_map if name=="firmware.map" else probe_board/name
                    shutil.copy2(source,dest/name)
                (dest/"stats.json").write_text(json.dumps(probe_stats,indent=2)+"\n")
            try:
                retry=recovery.select_probe_layout(probe_map.read_text(),probe_stats["flash_bytes"])
            except recovery.NoProbeLayout:
                print("Recovery sizing probe has no feasible whole-block layout",board,label,flush=True)
                if partition_count==16:
                    raise
                continue
            retry["lto_partitions_requested"]=partition_count
            break
        print("Recovery whole-partition retry",board,json.dumps(retry),flush=True)
        retry_dir=mode_dir/"layout-retry"
        retry_dir.mkdir(exist_ok=True)
        board_dir=retry_dir/board
        try:
            stats=baseline.build_target(retry_dir,cli,board,definitions,
                linker_script_factory=lambda bin_dir,directory: recovery.make_script(bin_dir,directory,
                    fill_named_text=retry["fill_named_text"],partitions=tuple(retry["partitions"])),
                recovery_lto_partitions=partition_count)
        except subprocess.CalledProcessError:
            archive_failed(archive, board_dir,mode,board+"-layout-retry-failed")
            raise
        stats["recovery_layout_retry"]=retry
    print(mode,board,json.dumps(stats),flush=True)
    if archive:
        dest=archive/mode/board
        dest.mkdir(parents=True)
        for pattern in ("*.elf","*.ino.hex","firmware.map"):
            paths=list((board_dir/"build").glob(pattern))
            assert len(paths)==1,(board,mode,pattern,paths)
            shutil.copy2(paths[0],dest/paths[0].name)
        for name in ("symbols.txt","sections.txt","properties.txt","disassembly.txt",
                     "properties-with-linker.txt","avr6-recovery.x"):
            if (board_dir/name).exists():
                shutil.copy2(board_dir/name,dest/name)
        (dest/"stats.json").write_text(json.dumps(stats,indent=2)+"\n")
        (dest/"verification.json").write_text(json.dumps({"status":"linked-unverified"})+"\n")
    dump=(board_dir/"disassembly.txt").read_text()
    symbols=(board_dir/"symbols.txt").read_text()
    vectors={int(m[2]):int(m[1],16) for m in re.finditer(
        r"^([0-9a-f]+)\s+(?:[0-9a-f]+\s+)?[TtWw]\s+__vector_(\d+)$",symbols,re.M)}
    assert vectors[32]==vectors[33],board
    if board in ("RF2000","RF2000v2"):
        assert vectors[32]==vectors[34],board
    for line in dump.splitlines():
        m=re.search(r"\bcall\s+0x([0-9a-f]+)\b|\brcall\s+[^;]+;\s*0x([0-9a-f]+)\b",line)
        if m:
            assert int(m[1] or m[2],16) not in vectors.values(),(board,line)
    if use_recovery:
        recovery.verify_image(board_dir)
        print("Linked renewal guard",board,json.dumps(timing.audit(dump)),flush=True)
        assert vectors[12] == vectors[2], (board,"WDT vector unexpectedly active")
        assert "heaterWatchdogEarlyInit" in dump, board
        start=dump.index("<heaterWatchdogEarlyInit>:")
        end=re.search(r"^[0-9a-f]+ <[^\n]+>:",dump[start+1:],re.M)
        assert end,board
        early=dump[start:start+1+end.start()]
        assert re.search(r"\bwdr\b",early),early
        assert not re.search(r"\bret[i]?\b",early),early
        # Timed reset configuration and feed locations are retained for
        # linked inspection; this does not prove oscillator or boot behavior.
        print("WDT reset-only candidate startup linked",board)
    print("Linked vector aliases and direct-call audit passed",board)
    if archive:
        (dest/"verification.json").write_text(json.dumps({"status":"verified"})+"\n")
    return stats, board_dir
