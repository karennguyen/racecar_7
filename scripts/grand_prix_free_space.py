def findLargestSpace(self, L, threshold):
	      	center = 0
	    	largestSpace = 0
	    	newL = []
	    
		#mark all legal areas as True
	    	for i in L:
			newL.append(i > threshold)
		    
		#go through and find the largest free space

		

	    	for i in range(len(newL)):
			if(newL[i]):
				count = 0
		    		while newL[i]:
					count += 1
		        		i += 1
					if(i>=len(newL)):
						break
		    		if(count > largestSpace):
		        		largestSpace = count
		        		center = (count / 2) + i
return center # a point out of 1081
