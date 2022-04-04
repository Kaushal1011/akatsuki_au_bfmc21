# function used to determine the next states 

def determine(x,maxlenx,maxleny):
    if x[0]<0 or x[0]>=maxlenx or x[1]<0 or x[1]>=maxleny:
        return True
    
# function used to determine whether the next states are relevant to the patch or not    
def possneigh(imgin,i,j,maxlenx,maxleny):
    poss_n=[(i+1,j),(i,j+1),(i+1,j+1),(i-1,j-1),(i-1,j),(i,j-1),(i-1,j+1),(i+1,j-1)]
    poss_n=[x for x in poss_n if not determine(x,maxlenx,maxleny)]
    return [x for x in poss_n if imgin[x[0]][x[1]]==0]

#bfs from ex2 modified to work on image
def bfs_4(graph,start):
    visited = [] # List to keep track of visited nodes.
    queue = []     #Initialize a queue
    visited.append(start)
    queue.append(start)

    while queue:
        s = queue.pop(0) 
        
        for neighbour in possneigh(graph,s[0],s[1],graph.shape[0],graph.shape[1]):
            if neighbour not in visited:
                visited.append(neighbour)
                queue.append(neighbour)
    return visited

#dfs from ex2 modified to work on image
def dfs_4(graph,start):       
        visited = []
 
        # Create a stack for DFS
        stack = []
 
        # Push the current source node.
        stack.append(start)
 
        while (len(stack)):
            # Pop a vertex from stack and print it
            s = stack[-1]
            stack.pop()
 
            # Stack may contain same vertex twice. So
            # we need to print the popped item only
            # if it is not visited.
            if s not in visited:
#                 print(s,end=' ')
                visited.append(s)
 
            # Get all adjacent vertices of the popped vertex s
            # If a adjacent has not been visited, then push it
            # to the stack.
#             if s not in graph.keys():
#                 continue
            for node in possneigh(graph,s[0],s[1],graph.shape[0],graph.shape[1]):
                if node not in visited:
                    stack.append(node)
        return visited