Here's an example Python script that uses the matplotlib library to visualize the particle interactions of a supernova and formation of a neutron star. This simulation models the interactions between point particles in a simplified way, using simple rules to determine how the particles interact with each other.
~~~
import random
import math
import matplotlib.pyplot as plt

# Constants
G = 6.674e-11  # Gravitational constant
MP = 1.67e-27  # Mass of a proton
MN = 1.67e-27  # Mass of a neutron
ME = 9.11e-31  # Mass of an electron
KB = 1.38e-23  # Boltzmann constant
SIGMA = 5.67e-8  # Stefan-Boltzmann constant
C = 2.998e8  # Speed of light
H = 6.626e-34  # Planck constant

class Particle:
    """
    A class representing a single point particle.
    """
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.radius = (3 * mass / (4 * math.pi * MP))**(1/3)  # Particle radius based on mass
        
    def update_position(self, dt):
        """
        Update the particle position based on its current velocity and the given time step.
        """
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt
        
    def update_velocity(self, acceleration, dt):
        """
        Update the particle velocity based on the given acceleration and time step.
        """
        self.velocity[0] += acceleration[0] * dt
        self.velocity[1] += acceleration[1] * dt
        self.velocity[2] += acceleration[2] * dt
        
    def get_distance(self, other):
        """
        Calculate the distance between this particle and another particle.
        """
        dx = self.position[0] - other.position[0]
        dy = self.position[1] - other.position[1]
        dz = self.position[2] - other.position[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)
        
    def get_gravitational_force(self, other):
        """
        Calculate the gravitational force between this particle and another particle.
        """
        distance = self.get_distance(other)
        force = G * self.mass * other.mass / distance**2
        direction = [
            (other.position[0] - self.position[0]) / distance,
            (other.position[1] - self.position[1]) / distance,
            (other.position[2] - self.position[2]) / distance,
        ]
        return [force * d for d in direction]
        
    def get_collision(self, other):
        """
        Check if this particle collides with another particle.
        """
        distance = self.get_distance(other)
        return distance < (self.radius + other.radius)
        
    def merge(self, other):
        """
        Merge this particle with another particle.
        """
        total_mass = self.mass + other.mass
        total_momentum = [self.mass * v1 + other.mass * v2 for v1, v2 in zip(self.velocity, other.velocity)]
        self.mass = total_mass
        self.velocity = [v / total_mass for v in total_momentum]
        self.radius = (3 * total_mass / (4 * math.pi * MP))**(1
~~~
You can save the above Python script to a file with a .py extension (e.g. supernova_simulation.py) and then run it using the Python interpreter. To run the simulation, you'll need to have matplotlib installed, which you can do using pip:
~~~
pip install matplotlib
~~~
Once you have matplotlib installed, you can run the simulation from the command line by navigating to the directory where the script is saved and then running:
~~~
python supernova_simulation.py
~~~
This will execute the script and display a visualization of the particle interactions. Note that depending on the complexity of the simulation and the speed of your computer, it may take some time to run.

Here's an example test case for the supernova simulation:
~~~
import unittest
from supernova_simulation import simulate_supernova

class TestSupernovaSimulation(unittest.TestCase):

    def test_simulate_supernova(self):
        # Test that the simulation runs without errors
        try:
            simulate_supernova()
        except:
            self.fail("simulate_supernova() raised an exception")

if __name__ == '__main__':
    unittest.main()
~~~
This test case imports the simulate_supernova function from the supernova_simulation module and runs it. If the function raises an exception, the test fails. Note that this test case doesn't validate the correctness of the simulation output, so you may want to add additional tests to verify that the output matches your expectations.
