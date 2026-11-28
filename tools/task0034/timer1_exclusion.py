"""Check Timer1 self-exclusion on the interprocedural graph from irq-mask-cfg."""
from collections import deque


def timer1_exclusion(graph, root, instructions):
    # OCIE1A is set on entry. At every reachable instruction boundary with
    # SREG.I set it must be clear, including boundaries after SREG restores.
    # No aliasing writes to MMIO via unrelated data pointers are assumed.
    def stored_bits(node, register):
        pending = list(graph.predecessors(node))
        visited = set()
        values = set()
        while pending:
            previous = pending.pop()
            if previous in visited:
                continue
            visited.add(previous)
            _, opcode, operands = instructions[previous[0]]
            args = operands.split(';')[0].strip().split(', ')
            if opcode in ('jmp', 'rjmp'):
                pending.extend(graph.predecessors(previous))
            elif args[0] == register and opcode in ('ori', 'andi', 'ldi'):
                bit = bool(int(args[1], 0) & 2)
                if opcode == 'ldi' or (opcode == 'ori') == bit:
                    values.add(bit)
                else:
                    raise ValueError(('OCIE1A not forced', hex(previous[0])))
            else:
                raise ValueError(('unknown OCIE1A value', hex(previous[0]), opcode))
        if not values:
            raise ValueError('unresolved TIMSK1 writer')
        return values

    pending = deque([(root, True)])
    visited = set()
    writers = {}
    while pending:
        node, enabled = pending.popleft()
        if (node, enabled) in visited:
            continue
        visited.add((node, enabled))
        pc, interruptible = node[0], node[-1]
        if interruptible and enabled:
            raise ValueError(('Timer1 self-preemption possible', hex(pc)))
        _, opcode, operands = instructions[pc]
        args = operands.split(';')[0].strip().split(', ')
        bits = (enabled,)
        if opcode == 'sts' and int(args[0], 0) == 0x6f:
            bits = stored_bits(node, args[1])
            writers.setdefault(hex(pc), set()).update(bits)
        for child in graph.successors(node):
            pending.extend((child, value) for value in bits)
    return {'statePairs': len(visited), 'writes': {k: sorted(v) for k,v in writers.items()}}
