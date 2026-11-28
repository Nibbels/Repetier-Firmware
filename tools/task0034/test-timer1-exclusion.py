"""Regression cases for SREG.I / OCIE1A interleavings and merged store sites."""
import unittest
import networkx as nx
from timer1_exclusion import timer1_exclusion


class Timer1GateTest(unittest.TestCase):
    def check(self, enables, mask_instruction, open_at_store=False):
        graph = nx.DiGraph()
        states = [(i, level) for i, level in enumerate((0, 0, int(open_at_store), int(enables)))]
        graph.add_edges_from(zip(states, states[1:]))
        instructions = [(2, "nop", ""), (2, *mask_instruction), (4, "sts", "0x006F, r24"), (2, "nop", "")]
        return timer1_exclusion(graph, states[0], instructions)

    def test_clear_before_open(self):
        self.assertEqual(self.check(True, ("andi","r24, 0xFD"))["writes"], {"0x2":[False]})

    def test_open_before_clear(self):
        with self.assertRaisesRegex(ValueError, "self-preemption"):
            self.check(True, ("andi","r24, 0xFD"), True)

    def test_set_before_open(self):
        with self.assertRaisesRegex(ValueError, "self-preemption"):
            self.check(True, ("ori","r24, 0x02"))

    def test_two_predecessors_share_store(self):
        graph = nx.DiGraph()
        states = [(i, 0) for i in range(5)] + [(5, 1)]
        graph.add_edges_from((states[a],states[b]) for a,b in ((0,1),(0,2),(1,3),(2,4),(4,3),(3,5)))
        instructions = [(2,"nop",""), (2,"andi","r24, 0xFD"), (2,"ori","r24, 0x02"),
                        (4,"sts","0x006F, r24"), (2,"rjmp",""), (2,"nop","")]
        with self.assertRaisesRegex(ValueError, "self-preemption"):
            timer1_exclusion(graph, states[0], instructions)

    def test_unknown_writer(self):
        with self.assertRaisesRegex(ValueError, "unknown OCIE1A"):
            self.check(False, ("mov","r24, r1"))


if __name__ == "__main__":
    unittest.main()
