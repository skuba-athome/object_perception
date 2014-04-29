import sys

filePtr = open("../learn/LocalizationTrain/" + sys.argv[1]  + ".train","r")
labels = []
name = []

for line in filePtr:
	label = line.split(" ")[0]
	if label in labels:
		continue
	labels.append(label)
	name.append(line.split("/")[1])

fileObject = open("../object.txt","w")
for i in range(len(labels)):
	fileObject.write(labels[i] + " " + name[i] + "\n")
	#print labels[i] + " : " + name[i]

