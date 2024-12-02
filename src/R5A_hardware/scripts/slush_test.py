import Slush

def main():
    try:
        print("Initializing SlushEngine board...")
        board = Slush.sBoard()

        print("Configuring motors...")
        motors = [Slush.Motor(i) for i in range(5)]
        for i, motor in enumerate(motors):
            motor.setMicroSteps(16)
            motor.setCurrent(1000, 1000)
            print(f"Motor {i + 1} configured.")

        print("Testing motor movements...")
        for i, motor in enumerate(motors):
            print(f"Moving Motor {i + 1} by 1000 steps...")
            motor.move(1000)
            print(f"Motor {i + 1} position: {motor.getPosition()}")

        print("SlushEngine test complete.")

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
