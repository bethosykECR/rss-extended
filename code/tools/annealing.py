# By Matthew O'Kelly (February 2018)
# with modifications by Alena Rodionova
#

import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
import os
import csv


def get_next_sample(currentSample, initSpace):
    
    # Trick 1: Normalize the sample (rescale to map to [0,1])
    # (This is all about sampler efficiency)
    # Goal is to be able to sample from multivariate standard normal...
    # and then convert back to this space.
    #print('\ncurrent sample:')
    #print(currentSample)

    sampleWidth = currentSample - initSpace[:,0]
    intervalWidths = initSpace[:,1] - initSpace[:,0]
    #print('\ninterval width:')
    #print(intervalWidths)
    scaledSample = sampleWidth/intervalWidths
    #print('\nscaled sample')
    #print(scaledSample)
    # Trick 2: Sample from multivariate normal and get a unit direction
    direction = np.random.normal(size=(np.size(intervalWidths,)))
    length = LA.norm(direction)
    direction = direction/length

    idx = direction>0
    temp = (1-scaledSample)/direction
    try:
        z_plus_upper_step = np.min(temp[idx])
    except ValueError:
        z_plus_upper_step = np.inf

    idx = direction<0
    try:
        z_minus_upper_step = np.min(np.abs(temp[idx]))
    except ValueError:
        z_minus_upper_step = np.inf

    idx = direction<0
    temp = (-scaledSample)/direction
    try:
        z_plus_lower_step = np.min(temp[idx])
    except ValueError:
        z_plus_lower_step = np.inf

    idx = direction>0
    try:
        z_minus_lower_step = np.min(np.abs(temp[idx]))
    except ValueError:
        z_minus_lower_step = np.inf
    z_minus_step = np.minimum(z_minus_lower_step, z_minus_upper_step)
    z_plus_step = np.minimum(z_plus_lower_step, z_plus_upper_step)

    z = np.random.uniform(0, 1.2)
    if (z>1):
        z=0.99

    if np.random.uniform() < z_minus_step/(z_plus_step+z_minus_step):
        z *= -z_minus_step
    else:
        z *= z_plus_step

    # Trick 5: Convert back to actual sample space, basically undo 'Trick 1'
    nextSample = (scaledSample + z*direction)*intervalWidths + initSpace[:,0]
    #print('\nNEXT sample')
    #print(nextSample)
    return nextSample

def accept_value(proposal, objective, temperature):
    accept = False
    # We always accept moves which improve the objective
    if (proposal < objective):
        accept = True
    # And sometimes, moves which don't:
    else:
        # As the temperature increases it becomes less likely that
        # we will accept a proposal which is less than the objective.
        # Early on, it is easier to 'figuratively climb hills',
        # although there is no gradient here.
        # As the process moves forward the mass of the distribution
        # becomes concentrated around the current best objective value
        threshold = np.exp((objective - proposal) *temperature)
        u = np.random.uniform(0.0, 1.0)
        if (u <= threshold):
            accept = True
    return accept

def plot(history):

    fig, ax = plt.subplots()
    ax.plot(history, '-', label='Run %d, Solution: %f'%((j+1),solution))
    ax.set_title('Hit and Run: Annealing Random Search, $x^2+y^2+z^2$')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Objective Value')
    ax.legend()
    plt.show()


def runFunc(compute_objective, currentSamples, initSpace, nruns, num_simult_runs, RES_FOLDER=False):
    
    xdim = initSpace.shape[0]

    accept_x_history = np.zeros((nruns+1, num_simult_runs, xdim))
    accept_x_history[0, :, :] = np.copy(currentSamples)

    x_history = np.zeros((nruns+1, num_simult_runs, xdim))
    x_history[0, :, :] = np.copy(currentSamples)

    
    accept_flags = []
    accept_flags.append(np.full((1, num_simult_runs), True))
   
    Y = []
    for i in range(num_simult_runs):
        #print(currentSamples[i])
        y = compute_objective(currentSamples[i])
        Y.append(y)
 
    objectives = np.copy(Y)
    solutions  = np.copy(Y)

    best_f_history = np.zeros((nruns+1, num_simult_runs))
    best_f_history[0,:] = np.copy(objectives)

    best_x_history = np.zeros((nruns+1, num_simult_runs, xdim))
    best_x_history[0, :, :] = np.copy(currentSamples)
    
    f_history = np.zeros((nruns+1, num_simult_runs))
    f_history[0,:] = np.copy(objectives)

    sol_x = np.copy(currentSamples)

    
    
    def saveToFile(i, scope, RES_FOLDER, flag='a'):
        var_list = ['best_x_history', 'best_f_history', 'x_history', 'f_history']
        if RES_FOLDER:
            for name in var_list:
                filename = os.path.join(RES_FOLDER, name+'.csv')
                with open(filename, flag) as file:
                    writer = csv.writer(file, delimiter=',')
                    val = eval(name, scope)
                    val = val[i].reshape(-1)
                    writer.writerow(val)

    
    saveToFile(0, locals(), RES_FOLDER, 'w')
    
    tsched = 100.
    for i in range(nruns):
        temperature = i*(1./tsched)
        #print ('Run: %d, Temperature: %.3f'% (i, temperature))
        nextSamples = []
        Z = []
        for j in range(num_simult_runs):
            nextSamples.append(get_next_sample(currentSamples[j], initSpace))
            z = compute_objective(nextSamples[j])
            Z.append(z)

        proposals = Z
        
        f_history[i+1,:] = np.copy(Z)
        x_history[i+1,:,:]= np.copy(nextSamples)

        accepts = []
        for j in range(num_simult_runs):
            accepts.append(accept_value(proposals[j],objectives[j],temperature))
            if accepts[j]:
                currentSamples[j] = nextSamples[j]
                objectives[j] = proposals[j]


            if (objectives[j] < solutions[j]):
                solutions[j] = objectives[j]
                sol_x[j] = currentSamples[j]

        accept_x_history[i+1,:,:] = np.copy(currentSamples)
        accept_flags.append(np.copy(accepts))

        best_f_history[i+1,:] = np.copy(solutions)
        best_x_history[i+1, :, :] = np.copy(sol_x)

        saveToFile(i+1, locals(), RES_FOLDER)


    return best_x_history, best_f_history, x_history, f_history, accept_x_history, accept_flags