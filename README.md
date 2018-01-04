Optimal Reciprocal Collision Avoidance
======================================

Original site: <http://gamma.cs.unc.edu/RVO2/>
Original repo: <https://github.com/snape/RVO2-CS>

This is almost a line-for-line port of the C# version of RV02 to pure Python 3. It is not meant to be peformant but simple.

Example:

```
from rvo.simulator import Simulator

simulator = Simulator()
id0 = simulator.add_agent(Vector2(0, 0))
id1 = simulator.add_agent(Vector2(0, 1))
id2 = simulator.add_agent(Vector2(1, 1))

goal = Vector2(0, 0)
goal0 = goal - simulator.agents_[id0].position_
goal1 = goal - simulator.agents_[id1].position_
goal2 = goal - simulator.agents_[id2].position_

while True:
    simulator.set_agent_pref_velocity(id0, goal0)
    simulator.set_agent_pref_velocity(id1, goal1)
    simulator.set_agent_pref_velocity(id2, goal2)

    simulator.step()
```

See [Circle](examples/circle.py) and [Blocks](examples/blocks.py) for more involved examples.
