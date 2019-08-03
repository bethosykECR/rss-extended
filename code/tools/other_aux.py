import csv

def write2csv(filename, x):
    with open(filename, 'a') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(x)