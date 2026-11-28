// Task0033: deterministic handoff for Task0032; not a new whole-system simulator.
function contract(){
  const F=16000000,baud=250000,frame=10,wire=baud/frame;
  const budgets={rx_normal:80,rx_error:100,tx_normal:80,tx_empty:64,entry_to_udr:40,ready_to_udr:512,nonpreemptible_design_guideline:400,nominal_frame_cycles:640,other_delay_budget:472,nominal_margin_cycles:128};
  const planningFraction=.8, planningBytes=wire*planningFraction;
  const round=n=>Math.round(n*1e6)/1e6;
  const rows=[];
  for(const rxBytes of [20,40,60])for(const v of [20,60,100]){
    rows.push({rx_bytes_per_move:rxBytes,speed_mm_s:v,wire_max_moves_s:round(wire/rxBytes),planning_moves_s:round(planningBytes/rxBytes),minimum_mean_segment_at_wire_limit_mm:round(v*rxBytes/wire),minimum_mean_segment_at_planning_budget_mm:round(v*rxBytes/planningBytes)});
  }
  const stress=[];
  for(const rxBytes of [20,40,60])for(const length of [.025,.05,.1,.2,.5])for(const v of [20,60,100]){
    const moves=v/length,rx=moves*rxBytes;
    const tx=moves*10;
    stress.push({rx_bytes_per_move:rxBytes,segment_mm:length,requested_mm_s:v,offered_moves_s:round(moves),offered_rx_bytes_s:round(rx),offered_rx_wire_pct:round(100*rx/wire),wire_max_mm_s:round(wire/rxBytes*length),planning_max_mm_s:round(planningBytes/rxBytes*length),classification:rx<=planningBytes?"within_planning_wire_budget":rx<=wire?"near_wire_limit":"exceeds_wire_limit",illustrative_uart_cpu_pct:round(100*(rx*budgets.rx_normal+tx*budgets.tx_normal+moves*budgets.tx_empty)/F)});
  }
  const assert=(x,m)=>{if(!x)throw Error(m)};
  assert(budgets.ready_to_udr+budgets.nominal_margin_cycles===budgets.nominal_frame_cycles,"reserve");
  assert(budgets.other_delay_budget+budgets.entry_to_udr===budgets.ready_to_udr,"deadline decomposition");
  assert(F*frame/baud===budgets.nominal_frame_cycles,"frame budget");
  assert(rows.find(r=>r.rx_bytes_per_move===40&&r.speed_mm_s===60).minimum_mean_segment_at_planning_budget_mm===.12,"normal threshold");
  assert(rows.find(r=>r.rx_bytes_per_move===40&&r.speed_mm_s===100).minimum_mean_segment_at_planning_budget_mm===.2,"fast threshold");
  assert(stress.find(r=>r.rx_bytes_per_move===40&&r.segment_mm===.05&&r.requested_mm_s===20).classification==="within_planning_wire_budget","slow dense case");
  assert(stress.find(r=>r.rx_bytes_per_move===40&&r.segment_mm===.05&&r.requested_mm_s===60).classification==="exceeds_wire_limit","dense case");
  assert(stress.find(r=>r.rx_bytes_per_move===40&&r.segment_mm===.1&&r.requested_mm_s===60).classification==="near_wire_limit","near line rate");
  return {schema:1,status:"owner_agreed_baud_and_proposed_engineering_budgets_not_qualified",clock_hz:F,baud,reference_baud:115200,optional_future_baud:500000,bits_per_frame:frame,wire_bytes_s:wire,normal_ubrr:3,u2x_ubrr:7,budgets_cycles:budgets,measured_isolated_candidate:{rx_normal_cycles:77,rx_read_complete_cycles:33,rx_error_paths:[67,80],tx_normal_cycles:71,tx_empty_cycles:61,rx_stack_incl_return:12,tx_stack_incl_return:11,ci_run:45941580801,artifact:15172592211},recommended_planning_fraction:planningFraction,planning_rx_bytes_s:planningBytes,rx_ring_usable:127,tx_ring_usable:63,empty_rx_ring_fill_ms:round(127/wire*1000),empty_tx_ring_drain_ms:round(63/wire*1000),saturated_rx_normal_budget_pct:100*wire*budgets.rx_normal/F,saturated_full_duplex_normal_budget_pct:100*wire*(budgets.rx_normal+budgets.tx_normal)/F,limits:"Assumed actual wire frame sizes 20/40/60 bytes (not measured slicer distributions), 80% wire planning allowance is an engineering scenario not a safety guarantee or firmware threshold. Illustrative UART CPU uses 10 ACK bytes and one 64-cycle empty TX interrupt per move, does not account for parser, actual error traffic, other ISRs or infeasible offered overload. Need true ACK/diagnostic byte counts. Minimum mean length is only a sustained wire supply boundary, not a hard per-move minimum, not a print speed qualification. No loss proof from nominal average or maximum single mask interval. Prove total ready-to-UDR response and FIFO occupancy for all admitted bursts, include clock tolerance, pending IRQ, deferral and initial backlog. No timing assumption overrides P0 or STEP/Advance safety.",rows,stress};
}
console.log(JSON.stringify(contract(), null, 2));
