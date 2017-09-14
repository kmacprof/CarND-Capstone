STATE_INIT = 0
STATE_UP = 1
STATE_DOWN = 2


class Twiddler:
    """
    A stateful implementation of the 'twiddle' algorithm from the lectures.

    To use, call twiddle each time you evaluate the objective function. It will
    update the parameters based on whether it was an improvement over the last
    call.
    """

    def __init__(self, values, deltas, growth=1.1, decay=0.9):
        self.values = values
        self.deltas = deltas
        self.parameter = 0
        self.growth = growth
        self.decay = decay
        self.best_error = float('inf')
        self.state = STATE_INIT

    def twiddle(self, last_error):
        if self.state == STATE_INIT:
            self.best_error = last_error
            self.__twiddle_up()

        elif self.state == STATE_UP:
            if last_error < self.best_error:
                self.best_error = last_error
                self.deltas[self.parameter] *= self.growth
                self.__twiddle_next_parameter_up()
            else:
                self.values[self.parameter] -= 2 * self.deltas[self.parameter]
                self.state = STATE_DOWN

        elif self.state == STATE_DOWN:
            if last_error < self.best_error:
                self.best_error = last_error
                self.deltas[self.parameter] *= self.growth
            else:
                self.values[self.parameter] += self.deltas[self.parameter]
                self.deltas[self.parameter] *= self.decay

            self.__twiddle_next_parameter_up()

    def __twiddle_next_parameter_up(self):
        self.parameter = (self.parameter + 1) % len(self.values)
        self.__twiddle_up()

    def __twiddle_up(self):
        self.values[self.parameter] += self.deltas[self.parameter]
        self.state = STATE_UP

    def total_delta(self):
        return sum(self.deltas)

# The original twiddle:
# def twiddle(tol=0.2):
#     p = [0, 0, 0]
#     dp = [1, 1, 1]
#     best_err = run(robot, p)
#
#     it = 0
#     while sum(dp) > tol:
#         print("Iteration {}, best error = {}".format(it, best_err))
#         for i in range(len(p)):
#             p[i] += dp[i]
#             best_err = run(robot, p)
#
#             if err < best_err:
#                 best_err = err
#                 dp[i] *= 1.1
#             else:
#                 p[i] -= 2 * dp[i]
#                 best_err = run(robot, p)
#
#                 if err < best_err:
#                     best_err = err
#                     dp[i] *= 1.1
#                 else:
#                     p[i] += dp[i]
#                     dp[i] *= 0.9
#         it += 1
#     return p
