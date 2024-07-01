def compute_mean(filename):
    with open(filename, 'r') as file:
        data = file.readlines()
        data = [float(item.strip()) for item in data]
        mean = sum(data) / len(data)
        return mean


meanWBC = compute_mean('WBCtime.txt')
meanSim = compute_mean('stepSimtime.txt')
print('mean WBC time', meanWBC)
print('mean Step Sim', meanSim)