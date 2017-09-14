import sys
import unittest

sys.path.insert(0, '..')
from twiddler import Twiddler


def rosenbrock(values):
    """
    https://en.wikipedia.org/wiki/Rosenbrock_function
    """
    return (1 - values[0])**2 + 100 * (values[1] - values[0]**2)**2


class TestTwiddler(unittest.TestCase):

    def test_twiddle_1d(self):
        twiddler = Twiddler([1], [0.1])
        self.assertEqual(1, twiddler.values[0])

        # First twiddle sets the baseline; then we twiddle up.
        twiddler.twiddle(13)
        self.assertAlmostEqual(1.1, twiddler.values[0])
        self.assertAlmostEqual(0.1, twiddler.deltas[0])

        # Suppose that did not help; next we twiddle down.
        twiddler.twiddle(13)
        self.assertAlmostEqual(0.9, twiddler.values[0])
        self.assertAlmostEqual(0.1, twiddler.deltas[0])

        # Suppose that still didn't help; next we decay delta and twiddle up.
        twiddler.twiddle(13)
        delta_1 = 0.9 * 0.1
        self.assertAlmostEqual(delta_1, twiddler.deltas[0])
        self.assertAlmostEqual(1 + delta_1, twiddler.values[0])

        # Suppose that did help; should grow delta and twiddle up.
        twiddler.twiddle(12)
        delta_2 = 1.1 * delta_1
        self.assertAlmostEqual(delta_2, twiddler.deltas[0])
        self.assertAlmostEqual(1 + delta_1 + delta_2, twiddler.values[0])

        # Suppose that didn't help; next we twiddle back down.
        twiddler.twiddle(12)
        self.assertAlmostEqual(delta_2, twiddler.deltas[0])
        self.assertAlmostEqual(1 + delta_1 - delta_2, twiddler.values[0])

        # Suppose that did help; should grow delta and twiddle up.
        twiddler.twiddle(11)
        delta_3 = 1.1 * delta_2
        self.assertAlmostEqual(delta_3, twiddler.deltas[0])
        self.assertAlmostEqual(1 + delta_1 - delta_2 + delta_3,
                               twiddler.values[0])

    def test_twiddle_2d_rosenbrock(self):
        """
        Make sure we can actually optimize something.
        """
        twiddler = Twiddler([2, 2], [0.2, 0.2])

        self.assertEqual(401, rosenbrock(twiddler.values))
        num_iterations = 0
        while num_iterations < 2000 and twiddler.total_delta() > 1e-3:
            twiddler.twiddle(rosenbrock(twiddler.values))
            num_iterations += 1

        self.assertLess(num_iterations, 2000)
        self.assertLess(abs(1 - twiddler.values[0]), 0.3)
        self.assertLess(abs(1 - twiddler.values[1]), 0.4)
        self.assertLess(rosenbrock(twiddler.values), 1)

if __name__ == '__main__':
    import rostest
    rostest.unitrun(
        'twist_controller',
        'test_twiddler', TestTwiddler)
