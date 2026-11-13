import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


class ShutdownTests(unittest.TestCase):
    def run_main(self, context_live):
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'payload_controller.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        events=[]
        def spin(node):
            raise RuntimeError('Subscription take after context shutdown')
        scope={'rclpy':NS(init=lambda:None,spin=spin,ok=lambda:context_live,
                          try_shutdown=lambda:events.append('shutdown')),
               'PayloadGait':lambda:NS(destroy_node=lambda:events.append('destroy')),
               'signal':NS(SIGINT=2,SIG_IGN=1,signal=lambda *args:None)}
        exec(compile(ast.Module(body=[main],type_ignores=[]),'<lifecycle test>','exec'),scope)
        if context_live:
            with self.assertRaises(RuntimeError): scope['main']()
        else:
            scope['main']()
        self.assertEqual(events,['destroy','shutdown'])

    def test_shutdown_race_exits_cleanly(self): self.run_main(False)
    def test_active_controller_exceptions_are_not_suppressed(self): self.run_main(True)


if __name__=='__main__': unittest.main()
