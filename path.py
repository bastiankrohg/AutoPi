import numpy as np
import random

def generate_straight_line_path(length, step_size, start_position=(0, 0)):
    """
    Generate a straight-line path.

    Parameters:
    - length (int): Number of steps in the straight line.
    - step_size (float): Distance of each step.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    x, y = start_position
    path = [(x, y)]

    for _ in range(length):
        y += step_size  # Move north/up
        path.append((x, y))

    return path

def generate_random_walk_path(steps, step_size, start_position=(0, 0)):
    """
    Generate a random walk path.

    Parameters:
    - steps (int): Number of steps in the random walk.
    - step_size (float): Distance of each step.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    path = [start_position]
    x, y = start_position
    directions = [(0, step_size), (step_size, 0), (0, -step_size), (-step_size, 0)]  # Up, Right, Down, Left

    for _ in range(steps):
        dx, dy = random.choice(directions)
        x += dx
        y += dy
        path.append((x, y))

    return path

def generate_spiral_pattern(step_size, num_turns, start_position=(0, 0)):
    """
    Generate a spiral pattern path.

    Parameters:
    - step_size (float): Distance of each step.
    - num_turns (int): Number of turns in the spiral.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    path = [start_position]
    x, y = start_position
    angle = 0

    for i in range(1, num_turns + 1):
        angle += np.pi / 4  # 45-degree turn per segment
        dx = step_size * i * np.cos(angle)
        dy = step_size * i * np.sin(angle)
        x += dx
        y += dy
        path.append((round(x), round(y)))

    return path

def generate_zigzag_pattern(step_size, width, height, start_position=(0, 0)):
    """
    Generate a zigzag pattern path.

    Parameters:
    - step_size (float): Distance of each step.
    - width (int): Width of the zigzag.
    - height (int): Height of the zigzag.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    path = [start_position]
    x, y = start_position

    for _ in range(height):
        # Move right
        x += step_size * width
        path.append((x, y))
        # Move up one step
        y += step_size
        path.append((x, y))
        # Move left
        x -= step_size * width
        path.append((x, y))
        # Move up one step
        y += step_size
        path.append((x, y))

    return path

def generate_sine_wave_path(amplitude, wavelength, total_distance, step_size=1, start_position=(0, 0)):
    """
    Generate a sine wave path.

    Parameters:
    - amplitude (float): The peak deviation of the wave.
    - wavelength (float): The distance over which the wave repeats.
    - total_distance (float): The total horizontal distance to cover.
    - step_size (float): The horizontal distance between each point.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    x_start, y_start = start_position
    x_values = np.arange(0, total_distance, step_size)
    y_values = amplitude * np.sin(2 * np.pi * x_values / wavelength)
    path = [(round(x_start + x), round(y_start + y)) for x, y in zip(x_values, y_values)]
    return path

def generate_expanding_square_path(step_size, num_layers, start_position=(0, 0)):
    """
    Generate an expanding square path starting from the center.

    Parameters:
    - step_size (float): The distance of each step.
    - num_layers (int): The number of layers to expand.
    - start_position (tuple): Starting position (x, y) of the path.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    path = [start_position]  # Start at the origin
    x, y = start_position
    directions = [(0, step_size), (step_size, 0), (0, -step_size), (-step_size, 0)]  # Up, Right, Down, Left
    dir_index = 0  # Start by moving up

    for layer in range(1, num_layers + 1):
        for _ in range(2):  # Two sides per layer length
            for _ in range(layer):
                dx, dy = directions[dir_index]
                x += dx
                y += dy
                path.append((x, y))
            dir_index = (dir_index + 1) % 4  # Turn to the next direction

    return path

if __name__ == "__main__":
    # Example usage

    # Generate a random walk path
    random_walk_path = generate_random_walk_path(steps=50, step_size=1, start_position=(5, 5))
    print("Random Walk Path:")
    print(random_walk_path)

    # Generate a spiral pattern path
    spiral_path = generate_spiral_pattern(step_size=1, num_turns=10, start_position=(5, 5))
    print("Spiral Pattern Path:")
    print(spiral_path)

    # Generate a zigzag pattern path
    zigzag_path = generate_zigzag_pattern(step_size=1, width=5, height=3, start_position=(5, 5))
    print("Zigzag Pattern Path:")
    print(zigzag_path)

    # Generate a sine wave pattern path
    sine_wave_path = generate_sine_wave_path(amplitude=5, wavelength=10, total_distance=50, step_size=1, start_position=(5, 5))
    print("Sine Wave Pattern Path:")
    print(sine_wave_path)

    # Generate an expanding square pattern path
    expanding_square_path = generate_expanding_square_path(step_size=1, num_layers=5, start_position=(0, 0))
    print("Expanding Square Pattern Path:")
    print(expanding_square_path)