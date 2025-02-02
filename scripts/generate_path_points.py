import numpy as np

def generate_circle_path_points(center, radius, point_distance):
    h, k = center  # Center of the circle
    theta_step = point_distance / radius  # Calculate angle step size
    theta = np.arange(0, 2 * np.pi, theta_step)  # Generate angles
    
    # Parametric equation of the circle
    x = h + radius * np.cos(theta)
    y = k + radius * np.sin(theta)
    
    # Combine x and y into a list of points
    points = np.column_stack((x, y))
    
    # Round points to 2 decimal places
    rounded_points = np.round(points, 2)
    return rounded_points

def generate_sinusoidal_path_points(amplitude, wavelength, length):
    # x = np.linspace(0, length, num=40)  # 500 points along the path
    x = np.arange(0, 2*np.pi, 0.1)
    y = amplitude * np.sin(2 * np.pi * x / wavelength)

    points = np.column_stack((x, y))
    rounded_points = np.round(points, 2)
    return rounded_points



# Example usage
center = (2.9, 2.9)
radius = 2.9
point_distance = 0.8

amplitude = 3  # Max turning radius
wavelength = 15  # Wavelength
length = 100   

# points = generate_circle_path_points(center, radius, point_distance)
points = generate_sinusoidal_path_points(amplitude, wavelength, length)
print(points)

