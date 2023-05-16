import time
def fps_timer(func):
    def wrapper(*args, **kwargs):
        # Initialize the FPS timer.
        prev_time = time.time()

        # Call the function and store the result.
        result = func(*args, **kwargs)

        # Calculate the FPS and print it.
        cur_time = time.time()
        if cur_time - prev_time > (0.0000001):
            fps = 1.0 / (cur_time - prev_time)
            print('FPS: {:.2f}'.format(fps))

        # Return the result of the function.
        return result

    # Return the decorated function.
    return wrapper
