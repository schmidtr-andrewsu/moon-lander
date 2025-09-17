# moon-lander

## How to develop

- Create virtual environment
```bash
python -m venv .venv 
```

- Source into the virtual environment:
```bash
source .venv/bin/activate
```

- Install dependencies
```bash
pip install -r requirements.txt
```

## Run the Program
```bash
python moon_lander.py
```

## Controls
- Thrust: ↑ / W / Space
- Rotate: ← / → or A / D
- Strafe (RCS): Q (left) / E (right)
- Retry same terrain: R
- New terrain (new seed): N

## Landing requirements
- Keep your angle within ±8° of upright.
- Keep horizontal speed low (|Vx| ≤ ~1.8).
- Keep vertical speed gentle (Vy ≤ ~2.5).
- Touch down with both feet inside the green pad.
