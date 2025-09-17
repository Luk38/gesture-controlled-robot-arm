import random
import argparse

baselines = ["PS4", "Spacemouse"]
neue_Systeme = ["OSC Pose", "OSC with Linear Velocity"]

alle = baselines + neue_Systeme

def zufallsreihenfolge(systeme, seed=None):
    rnd = random.Random(seed) if seed is not None else random
    return rnd.sample(systeme, k=len(systeme))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gibt eine zufällige Reihenfolge der Systeme aus.")
    parser.add_argument("--seed", type=int, default=None, help="Optionaler Seed für reproduzierbare Reihenfolge.")
    args = parser.parse_args()

    reihenfolge = zufallsreihenfolge(alle, seed=args.seed)
    for i, name in enumerate(reihenfolge, 1):
        print(f"{i}. {name}")