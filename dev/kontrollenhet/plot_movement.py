import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from math import cos, sin, radians


def plot_movements(movements : list):
    """
    Create a plot of the robots movements. Each line is drawn individually, to separate them by color.

    Args:
        :param movements (list): List of (current_angle, distance_since_start) of the robots movements

    Returns:
        :return (plt): A pyplot of the robots movements.
    """
    # Start first line at (0,0). (The second pair will be popped before use)
    x_plot = [0, 0]
    y_plot = [0, 0]

    last_distance = 0
    for angle, distance in movements:
        # Calculate distance moved since last savepoint
        current_distance = distance - last_distance

        # Add lines ending (x2,y2) from last position (x1,y1)
        x_plot.append(x_plot[-1] + current_distance * sin(radians(angle)))
        y_plot.append(y_plot[-1] + current_distance * cos(radians(angle)))

        # Remove old unneded (x0,y0)-position
        y_plot.pop(0)
        x_plot.pop(0)

        # Add current line to plot
        plt.plot(x_plot, y_plot)
        last_distance = distance

    # Extends the plot by 100 millimeters in each direction to avoid plots disapperaing into the borders.
    x_lim_lower, x_lim_upper = plt.xlim()
    plt.xlim(x_lim_lower-100, x_lim_upper + 100)
    y_lim_lower, y_lim_upper = plt.ylim()
    plt.ylim(y_lim_lower-100, y_lim_upper + 100)

    return plt


if __name__ == "__main__":
    # Original measurements
    # movements = [(0, 451.0), (-88, 1992.0), (-169, 2702.0), (-251, 3285.0), (-170, 3509.0), (-257, 3947.0), (-351, 4270.0), (-441, 5198.0), (-370, 5570.0), (-294, 5893.0), (-194, 5816.0)]

    # Manually adjusted to 0,90,180 etc.
    movements = [(0, 451.0), (-90, 1992.0), (-180, 2702.0), (-270, 3285.0), (-180, 3509.0), (-270, 3947.0), (-360, 4270.0), (-450, 5198.0), (-360, 5570.0), (-270, 5893.0)]

    # Manually adjusted to wall sizes of 400mm.
    # movements = [(0, 400.0), (-90, 2000.0), (-180, 2800.0), (-270, 3200.0), (-180, 3600.0), (-270, 4000.0), (-360, 4400.0), (-450, 5200.0), (-360, 5600.0), (-270, 6000.0)]

    plot_movements(movements).show()