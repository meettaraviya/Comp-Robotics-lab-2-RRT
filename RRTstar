


#Firstly, you map a cost to each vertex
#E is defined here as {s_parent, s_daughter} , just to fit the pseudocode (check messanger) 


#pull the plug when it runs for too long
max_iteration = 10000

#For rewiring: consider all vertex within this radius 

#set search_radius <= rrt drive time, explanation below (see "@@@")
rrt_search_radius = 1


def RRT_star(s_initial, target, obstacles):
    iter_counter = 0
    S = [s_initial]
    
    #0 cost for initial S
    cost = {s_initial: 0.0} 
    
    E = []
    # target = (x_target, y_target, h_target)
    # obstacles = [ (p1, p2, p3, p4) ]
    while (iter_counter < max_iteration):
    
        
    #s_rand random state:
        s_rand = (np.random.rand()*map_width, np.random.rand()*map_length, np.random.rand()*2*np.pi) 

        """#if the
            if (in_obstacle(s_rand) = True):
                #try again: 
                continue"""
           
        s_nearest = nearest_neighbor(S, s_rand)

        s_new, action = drive_towards(s_nearest, s_rand)
        
         
        if (check_collision(s_nearest, s_new, action, obstacles) == False):
            
        #find a set S neighbour within certain radius 
            s_neighbors = findNeighbors(S, s_new, rrt_search_radius)

        #check if you accidentally go into target while driving
            found_target, new_action = check_target(s_nearest, s_new, action, obstacles)
            if found_target:
                break
            

#!!! THIS LINE BELOW NOT IN THE PSEUDOCODE    
#prescribe a cost to s_new
#do you update costs here? 
            cost[s_new] = metric(s_nearest, s_new)

            
        # V <- V U (s_new)
            S.append(s_new)
            
        #x_min <- x_nearest
            s_min = s_nearest
        
        #c_min <- Cost(x_nearest) + c(Line(x_nearest, x_new))
            c_min = cost[s_nearest] + metric(s_nearest, s_new)
        
        #connect along a minimum cost path:
            for s_near in s_neighbors:
                
# "@@@" 
#SIMPLIFICATION here: 
# in theory you drive again here, and you might stop before you complete the journey from s_min -> s_new
#BUT:  set searchradius <= rrt drive time
#to avoid stopping in the middle of journey!

                if (collisionfree(s_near,s_new) == True and (cost[s_near] + metric(s_near, s_new) < c_min)):
                    s_min = s_near
                    c_min = cost[s_near] + metric(s_near,s_new)
                    
        # E <- E U {(x_min, x_new)}
            E.append((s_min,s_new))

            
#!!! THIS LINE NOT IN THE PSEUDOCODE    
#prescribe a cost to s_new
#do you update costs here? 

            cost[s_new] = cost[s_min] + metric(s_min, s_new)
    
    
        #rewire the tree
            for s_near in s_neighbors:
#"@@@" same as above
                if ( collisionfree(s_new,s_near) and (cost[s_new] + metric(s_new, s_near) < cost[s_near])):
                    s_parent = Parent(s_near)
                    
                    
             
            #REMOVE OLD PARENT, GET NEW PARENT
            # E <- (E\{(x_parent,x_near)}) U {x_new,x_near}
            #english: E without (x_parent,x_near)  + (x_new,x_near)
                
                E.append((s_new,s_near))
                E.remove((s_parent,s_near))
                    
#!!! The line below IS NOT in Pseudocode!
            #!!! update cost of x_near, since x_near got a new dad
                cost[s_near] = cost[s_new]  + metric(s_new,s_near)
                    
 
        
                    
            #E.append( (s_nearest, s_new, new_action) )

        display_edge( (s_nearest, s_new, new_action) )

        #finally, increase the counter            
        iter_counter+=1
#RENEW X' 
#CHECK YOUR DRIVING when rewiring
                    

    print_path(S, E, s_initial, target, obstacles)

    return S,E
max_iteration = 10000

#For rewiring: consider all vertex within this radius 

#set search_radius <= rrt drive time, explanation below (see "@@@")
rrt_search_radius = 1


def RRT_star(s_initial, target, obstacles):
    iter_counter = 0
    S = [s_initial]
    
    #0 cost for initial S
    cost = {s_initial: 0.0} 
    
    E = []
    # target = (x_target, y_target, h_target)
    # obstacles = [ (p1, p2, p3, p4) ]
    while (iter_counter < max_iteration):
    
        
    #s_rand random state:
        s_rand = (np.random.rand()*map_width, np.random.rand()*map_length, np.random.rand()*2*np.pi) 

        """#if the
            if (in_obstacle(s_rand) = True):
                #try again: 
                continue"""
           
        s_nearest = nearest_neighbor(S, s_rand)

        s_new, action = drive_towards(s_nearest, s_rand)
        
         
        if (check_collision(s_nearest, s_new, action, obstacles) == False):
            
        #find a set S neighbour within certain radius 
            s_neighbors = findNeighbors(S, s_new, rrt_search_radius)

        #check if you accidentally go into target while driving
            found_target, new_action = check_target(s_nearest, s_new, action, obstacles)
            if found_target:
                break
            

#!!! THIS LINE NOT IN THE PSEUDOCODE    
#prescribe a cost to s_new
#do you update costs here? 
            cost[s_new] = metric(s_nearest, s_new)

            
        # V <- V U (s_new)
            S.append(s_new)
            
        #x_min <- x_nearest
            s_min = s_nearest
        
        #c_min <- Cost(x_nearest) + c(Line(x_nearest, x_new))
            c_min = cost[s_nearest] + metric(s_nearest, s_new)
        
        #connect along a minimum cost path:
            for s_near in s_neighbors:
                
# "@@@" 
#SIMPLIFICATION here: 
# in theory you drive again here, and you might stop before you complete the journey from s_min -> s_new
#BUT:  set searchradius <= rrt drive time
#to avoid stopping in the middle of journey!

                if (collisionfree(s_near,s_new) == True and (cost[s_near] + metric(s_near, s_new) < c_min)):
                    s_min = s_near
                    c_min = cost[s_near] + metric(s_near,s_new)
                    
        # E <- E U {(x_min, x_new)}
            E.append((s_min,s_new))

            
#!!! THIS LINE NOT IN THE PSEUDOCODE    
#prescribe a cost to s_new
#do you update costs here? 

            cost[s_new] = cost[s_min] + metric(s_min, s_new)
    
    
        #rewire the tree
            for s_near in s_neighbors:
#"@@@" same as above
                if ( collisionfree(s_new,s_near) and (cost[s_new] + metric(s_new, s_near) < cost[s_near])):
                    s_parent = Parent(s_near)
                    
                    
             
            #REMOVE OLD PARENT, GET NEW PARENT
            # E <- (E\{(x_parent,x_near)}) U {x_new,x_near}
            #english: E without (x_parent,x_near)  + (x_new,x_near)
                
                E.append((s_new,s_near))
                E.remove((s_parent,s_near))
                    
#!!! The line below IS NOT in Pseudocode!
            #!!! update cost of x_near, since x_near got a new dad
                cost[s_near] = cost[s_new]  + metric(s_new,s_near)
                    
 
        
                    
            #E.append( (s_nearest, s_new, new_action) )

        display_edge( (s_nearest, s_new, new_action) )

        #finally, increase the counter            
        iter_counter+=1
#RENEW X' 
#CHECK YOUR DRIVING when rewiring
                    

    print_path(S, E, s_initial, target, obstacles)

    return S,E
