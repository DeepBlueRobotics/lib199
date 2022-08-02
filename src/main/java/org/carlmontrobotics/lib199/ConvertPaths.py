import csv
import glob
import os
for filename in glob.glob("../../../../deploy/PathWeaver/Paths/*.path"):
	pathCSV = csv.reader(open(filename,"r"))

	path = [row for row in pathCSV]
	original = csv.writer(open(filename[:-5]+"_original"+filename[-5:],"w",newline=''))
	for row in path:
		original.writerow(row)
	firstRow = None
	i = 0

	for row in path:

		if i > 0:
			row[:4] = [float(x) for x in row[:4]]
			if i == 1: 
				firstRow = row
				
			else:

				row[0] -= firstRow[0]
				row[1] -= firstRow[1]
		i+=1
		
	path[1][:2] = [0,0]
	for row in path[1:]:
		row[:4] = [round(x,3) for x in row[:4]]
	w = csv.writer(open(filename,"w",newline=""))		
	for row in path:
		w.writerow(row)
	print("Converted " + filename)


