"""Inventory explicit MMIO register accesses reached by the AVR graph walkers."""
def register_site(pc, opcode, operands):
    fields = operands.split(';')[0].strip().split(', ')
    if opcode in ('sei', 'cli', 'reti') or (opcode in ('bset', 'bclr') and fields == ['7']):
        return (hex(pc), 'SREG.I', opcode)
    if opcode not in ('lds', 'sts', 'in', 'out'):
        return None
    address = int(fields[1 if opcode in ('in', 'lds') else 0], 0)
    if opcode in ('in', 'out'):
        address += 0x20
    if address == 0x5f:
        return (hex(pc), 'SREG', opcode)
    if 0x6e <= address <= 0x73:
        return (hex(pc), f'TIMSK{address - 0x6e}', opcode)
    return None
