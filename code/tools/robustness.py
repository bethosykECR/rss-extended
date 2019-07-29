import csv

IDX_rob = 0
IDX_collision = 1
IDX_ego_velocity = 2


def distance(x, thr):
	# POSITIVE (TRUE) if x>=thr
    return [x-thr for x in x]


def always(x):
    return min(x)


def eventually(x):
    return max(x)


def alwaysDistance(x, thr=0.0):
	rob_traj = distance(x, thr)
	rob = always(rob_traj)
	return rob 


def readFile(file_name):
    data = []
    file = open(file_name)
    reader = csv.reader(file, quoting=csv.QUOTE_NONNUMERIC)
    for row in reader: # each row is a list
        data.append(row)

    file.close()
    return data


def column(data, i):
    return [row[i] for row in data]


def evaluateRobustness(traj):
    rob = alwaysDistance(traj)
    return rob


def getRobustness(file_name):
    data = readFile(file_name)
    traj_rob = column(data, IDX_rob)
    rob = evaluateRobustness(traj_rob)
    print('^^^^^^^^^^^^^^^^^^^^^^^^^^^')
    print('ROBUSTNESS = %.3f' % rob)
    return rob


def getRobustnessEtc(file_name):
    data = readFile(file_name)
    
    traj_rob = column(data, IDX_rob)
    traj_collision = column(data, IDX_collision)
    
    rob = evaluateRobustness(traj_rob)
    collision = max(traj_collision)
    print('ROBUSTNESS = %.3f, COLLISION = %i' % (rob, collision))
    return rob, collision