from picarx import Picarx
px = Picarx()

def main():
    print("Stopping Car...")
    px.forward(0)
    print("Car stopped, exiting program...")
    return
if __name__ == "__main__":
    main()
