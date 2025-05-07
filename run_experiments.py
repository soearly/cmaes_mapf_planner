import numpy as np
from mapf_planner import mapf_planner, import_mapf_instance

class CMAES:
    def __init__(self, mean, sigma, population_size=None, seed=None):
        self.rng = np.random.RandomState(seed)
        self.n = len(mean)
        self.mean = np.array(mean, dtype=np.float64).copy()
        self.sigma = sigma
        self.lambda_ = population_size if population_size else 4 + int(3 * np.log(self.n))
        self.mu = self.lambda_ // 2
        self.weights = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights /= np.sum(self.weights)
        self.mueff = 1 / np.sum(np.square(self.weights))
        self.cc = 4 / (self.n + 4)
        self.cs = (self.mueff + 2) / (self.n + self.mueff + 5)
        self.c1 = 2 / ((self.n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) /
                       ((self.n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0, np.sqrt((self.mueff - 1) /
                       (self.n + 1)) - 1) + self.cs
        self.pc = np.zeros(self.n)
        self.ps = np.zeros(self.n)
        self.B = np.eye(self.n)
        self.D = np.ones(self.n)
        self.C = np.eye(self.n)
        self.invsqrtC = np.eye(self.n)
        self.solutions = []
        self.generation_counter = 0
        self.eigeneval = 0

    def ask(self, number=None):
        if number is None:
            number = self.lambda_
        self.solutions = []
        for _ in range(number):
            z = self.rng.randn(self.n)
            y = self.B @ (self.D * z)
            x = self.mean + self.sigma * y
            self.solutions.append((x, y, z))
        return [s[0] for s in self.solutions]

    def tell(self, function_values):
        assert len(function_values) == len(self.solutions)
        idx_vals = sorted(enumerate(function_values), key=lambda x: x[1])
        idx_sorted = [idx for idx, _ in idx_vals[:self.mu]]
        x_selected = [self.solutions[i][0] for i in idx_sorted]
        y_selected = [self.solutions[i][1] for i in idx_sorted]
        old_mean = self.mean.copy()
        self.mean = np.sum([self.weights[i] * x_selected[i] for i in range(self.mu)], axis=0)
        self.ps = (1 - self.cs) * self.ps + np.sqrt(self.cs * (2 - self.cs) * self.mueff) * (
            self.invsqrtC @ ((self.mean - old_mean) / self.sigma)
        )
        hsig = int(np.linalg.norm(self.ps) /
                   np.sqrt(1 - (1 - self.cs) ** (2 * (self.generation_counter + 1))) <
                   (1.4 + 2 / (self.n + 1)) * np.sqrt(self.n))
        self.pc = (1 - self.cc) * self.pc + hsig * \
            np.sqrt(self.cc * (2 - self.cc) * self.mueff) * ((self.mean - old_mean) / self.sigma)
        weighted_ys = np.array([self.weights[i] * y_selected[i] for i in range(self.mu)])
        rank_one = self.c1 * np.outer(self.pc, self.pc)
        rank_mu = self.cmu * sum(
            self.weights[i] * np.outer(y_selected[i], y_selected[i]) for i in range(self.mu)
        )
        self.C = (1 - self.c1 - self.cmu) * self.C + rank_one + rank_mu
        self.sigma *= np.exp((self.cs / self.damps) *
                             (np.linalg.norm(self.ps) / np.sqrt(self.n) - 1))
        self.generation_counter += 1
        if self.generation_counter - self.eigeneval > self.lambda_ / (self.c1 + self.cmu) / self.n / 10:
            self.eigeneval = self.generation_counter
            self.C = np.triu(self.C) + np.triu(self.C, 1).T
            D2, B = np.linalg.eigh(self.C)
            idx = np.argsort(D2)[::-1]
            self.D = np.sqrt(D2[idx])
            self.B = B[:, idx]
            self.invsqrtC = self.B @ np.diag(1 / self.D) @ self.B.T

    def result(self):
        return self.mean, self.sigma

# ---------------- CMA-ES MAPF Integration ----------------

map_file = 'instances/test_1.txt'  # Update this path
my_map, starts, goals = import_mapf_instance(map_file)

dim = len(goals)
initial_mean = np.zeros(dim)
initial_sigma = 1.0
cmaes = CMAES(mean=initial_mean, sigma=initial_sigma)

n_generations = 100
for generation in range(n_generations):
    lamda = cmaes.ask()
    fitness_values = mapf_planner(lamda, my_map, starts, goals, disjoint=False)
    cmaes.tell(fitness_values)

    if generation % 10 == 0:
        best_fitness = min(fitness_values)
        mean, sigma = cmaes.result()
        print(f"Generation {generation}: Best fitness = {best_fitness:.6e}, Sigma = {sigma:.6e}")

mean, sigma = cmaes.result()
print(f"\nFinal solution: {mean}")
print(f"Final step size: {sigma:.6e}")
