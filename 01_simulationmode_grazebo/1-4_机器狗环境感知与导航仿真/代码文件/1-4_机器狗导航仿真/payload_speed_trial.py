"""Opt-in 0.025 m/s trial, without modifying the installed 6-3 package.

The speed envelope and bounded COM reference feedback are changed in memory.
The known v5.3.8 input shape is checked before compiling;
all joint, cadence, support, tracking and attitude checks are retained verbatim.
This is a candidate envelope, not a claim of physical acceptance.
"""
import ast
import inspect
import textwrap

MAX_TRIAL_SPEED = .025


def recenter_error(position, reference, measured_error, dx, dy):
    """Subtract the already-commanded normal shift before adding another one.

    The measured COM error is a desired body displacement, not a displacement
    to integrate repeatedly on top of an old virtual pose. Preserve requested
    motion parallel to the next support line; the transfer rate stays bounded.
    """
    projection = (-(reference[0]-position[0])*dy +
                  (reference[1]-position[1])*dx)/max(dx*dx+dy*dy, 1e-12)
    return [measured_error[0]+dy*projection, measured_error[1]-dx*projection]


def controller_tree(source, filename):
    tree = ast.parse(source, filename=filename)
    version = [n.value.value for n in tree.body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == 'VERSION' for t in n.targets)
               and isinstance(n.value, ast.Constant)]
    if version != ['lite3-only-v5.3.8-experimental-trot']:
        raise ValueError('Speed trial requires the reviewed v5.3.8 controller')
    gait = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Gait')
    init = next(n for n in gait.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    changed = 0
    for n in ast.walk(init):
        if not isinstance(n, ast.Compare):
            continue
        for left, op, right in zip([n.left]+n.comparators, n.ops, n.comparators):
            if (isinstance(left, ast.Attribute) and isinstance(left.value, ast.Name)
                    and left.value.id == 'self' and left.attr == 'trot_speed'
                    and isinstance(op, ast.LtE) and isinstance(right, ast.Constant)
                    and right.value == .020):
                right.value = MAX_TRIAL_SPEED
                changed += 1
    if changed != 1:
        raise ValueError('Unexpected controller speed validation; no trial loaded')
    return ast.fix_missing_locations(tree)


def planner_class(original, speed_limit, recenter_gain=3.):
    if not 0 < speed_limit <= MAX_TRIAL_SPEED:
        raise ValueError('Invalid payload trial speed limit')
    if not 2. <= recenter_gain <= 3.:
        raise ValueError('Invalid measured COM recenter gain')
    tree = ast.parse(textwrap.dedent(inspect.getsource(original.update)))
    changed = [0, 0, 0, 0]

    class SpeedBudget(ast.NodeTransformer):
        def visit_Call(self, node):
            node = self.generic_visit(node)
            if (isinstance(node.func, ast.Name) and node.func.id == 'clamp'
                    and len(node.args) == 3 and ast.dump(node.args[0]) == "Name(id='vx', ctx=Load())"
                    and isinstance(node.args[1], ast.UnaryOp)
                    and isinstance(node.args[1].op, ast.USub)
                    and isinstance(node.args[1].operand, ast.Constant)
                    and node.args[1].operand.value == .020
                    and isinstance(node.args[2], ast.Constant) and node.args[2].value == .020):
                node.args[1].operand = ast.Constant(speed_limit)
                node.args[2] = ast.Constant(speed_limit)
                changed[0] += 1
            return node

        def visit_Assign(self, node):
            node = self.generic_visit(node)
            # Match the complete original translation/rotation budget exactly.
            expected = ast.parse('scale = min(1.0, .020/max(abs(vx)+.30*abs(wz), 1e-12))').body[0]
            if ast.dump(node) == ast.dump(expected):
                node.value.args[1].left = ast.Constant(speed_limit)
                changed[1] += 1
            expected_recenter = ast.parse('scale = min(2.*dt, .020*dt/max(distance, 1e-12))').body[0]
            if ast.dump(node) == ast.dump(expected_recenter):
                # Faster convergence inside the unchanged 20 mm/s transfer
                # bound; retain the 5 mm / 60 ms gate and original deadline.
                node.value.args[0].left = ast.Constant(float(recenter_gain))
                changed[2] += 1
            expected_error = ast.parse('error = [a[0]+u*dx-com[0], a[1]+u*dy-com[1]]').body[0]
            if ast.dump(node) == ast.dump(expected_error):
                node.value = ast.Call(func=ast.Name(id='_payload_recenter_error', ctx=ast.Load()),
                                      args=[ast.Name(id='position',ctx=ast.Load()),
                                            ast.Attribute(value=ast.Name(id='self',ctx=ast.Load()),
                                                          attr='reference',ctx=ast.Load()),
                                            node.value, ast.Name(id='dx',ctx=ast.Load()),
                                            ast.Name(id='dy',ctx=ast.Load())], keywords=[])
                changed[3] += 1
            return node

    tree = SpeedBudget().visit(tree)
    if changed != [1, 1, 1, 1]:
        raise ValueError('Unexpected trot planner velocity budget; no trial loaded')
    scope = {}
    globals_ = dict(original.update.__globals__, _payload_recenter_error=recenter_error)
    exec(compile(ast.fix_missing_locations(tree), '<6-4 payload speed trial>', 'exec'), globals_, scope)
    return type('PayloadSpeedTrialTrot', (original,), {'update': scope['update']})
