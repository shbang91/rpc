import optuna


def objective(trial):
    x = trial.suggest_float("x", -10, 10)
    return (x - 2) ** 2


if __name__ == "__main__":
    study = optuna.load_study(
        study_name="distributed-example", storage = "mysql://carlitos:Sentis2001?@localhost/example")

    study.optimize(objective, n_trials=1000)