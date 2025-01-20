import numpy as np
import matplotlib.pyplot as plt
import random

def generate_sine_wave_path(amplitude, wavelength, total_distance, step_size=1):
    """
    Generate a sine wave path.

    Parameters:
    - amplitude (float): The peak deviation of the wave.
    - wavelength (float): The distance over which the wave repeats.
    - total_distance (float): The total horizontal distance to cover.
    - step_size (float): The horizontal distance between each point.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    x_values = np.arange(0, total_distance, step_size)
    y_values = amplitude * np.sin(2 * np.pi * x_values / wavelength)
    path = list(zip(x_values, y_values))
    return path

def generate_expanding_square_path(step_size, num_layers):
    """
    Generate an expanding square path starting from the center.

    Parameters:
    - step_size (float): The distance of each step.
    - num_layers (int): The number of layers to expand.

    Returns:
    - list of tuples: A list containing (x, y) coordinates of the path.
    """
    path = [(0, 0)]  # Start at the origin
    x, y = 0, 0
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

def visualize_path(path, title="Path Visualization"):
    """
    Visualize the generated path.

    Parameters:
    - path (list of tuples): The (x, y) coordinates of the path.
    - title (str): The title of the plot.
    """
    x_coords, y_coords = zip(*path)
    plt.figure(figsize=(8, 8))
    plt.plot(x_coords, y_coords, marker='o')
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

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

if __name__ == "__main__":

    # Generate a random walk path
    random_walk_path = generate_random_walk_path(steps=50, step_size=1, start_position=(5, 5))
    print("Random Walk Path:")
    print(random_walk_path)

    # Generate a spiral pattern path
    spiral_path = generate_spiral_pattern(step_size=1, num_turns=10, start_position=(8, 5))
    print("Spiral Pattern Path:")
    print(spiral_path)

    # Generate a zigzag pattern path
    zigzag_path = generate_zigzag_pattern(step_size=1, width=5, height=3, start_position=(5, 12))
    print("Zigzag Pattern Path:")
    print(zigzag_path)
