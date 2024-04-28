import time


def move_straight(distance, velocity):
        time_limit = distance / velocity
        starting_time = time.time()

        while True:
          #RosBot.go_forward()
          print(f"start_time:{starting_time}")
          elapsed_time = time.time() - starting_time
          print(f"elapsed_time:{elapsed_time}")
          if elapsed_time >= time_limit:
            break

if __name__ == "__main__":
    move_straight(12, 2)