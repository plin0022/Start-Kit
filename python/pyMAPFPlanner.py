import MAPF

from typing import Dict, List, Tuple
from queue import PriorityQueue


#0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:
    def __init__(self,env=None) -> None:
        self.env=env
    
        print("pyMAPFPlanner initialized!")

    def initialize(self,preprocess_time_limit:int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        pass
        print("planner initialize done" )
        # raise NotImplementedError()

    def plan(self,time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

        # example of only using single-agent search

        actions=[ MAPF.Action.W for i in range(len(self.env.curr_states))]
        for i in range(0,self.env.num_of_agents):
            print("start plan for agent ",i,end=" ")
            path=[]
            if len(self.env.goal_locations[i])==0:
                print(i," does not have any goal left",end=" ")
                path.append((self.env.curr_states[i].location,self.env.curr_states[i].orientation))
            else:
                print( " with start and goal: ",end=" ")
                path=self.single_agent_plan(self.env.curr_states[i].location,self.env.curr_states[i].orientation,self.env.goal_locations[i][0][0])

            print("current location:",path[0][0],"current direction: ",path[0][1])
            if path[0][0]!=self.env.curr_states[i].location:
                actions[i]=MAPF.Action.FW
            elif path[0][1]!=self.env.curr_states[i].orientation:
                incr=path[0][1]-self.env.curr_states[i].orientation
                if incr==1 or incr==-3:
                    actions[i]=MAPF.Action.CR
                elif incr==-1 or incr==3:
                    actions[i]=MAPF.Action.CCR
        # print(actions)
        return actions
        # print("python binding debug")
        # print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        # raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")


    def single_agent_plan(self, start:int,start_direct:int,end:int):
        print(start,start_direct)
        path=[]
        # AStarNode (u,dir,t,f)
        open_list=PriorityQueue()
        s=(start,start_direct,0,self.getManhattanDistance(start,end))
        open_list.put(s,0)
        all_nodes=dict()
        close_list=set()
        parent={(start,start_direct):None}
        all_nodes[start*4+start_direct]=s
        while not open_list.empty():
            curr=open_list.get()
            close_list.add(curr[0]*4+curr[1])
            if curr[0]==end:
                curr=(curr[0],curr[1])
                while curr!=None:
                    path.append(curr)
                    curr=parent[curr]
                path.pop()
                path.reverse()
                
                break
            neighbors=self.getNeighbors(curr[0],curr[1])
            # print("neighbors=",neighbors)
            for neighbor in neighbors:
                if (neighbor[0]*4+neighbor[1])  in close_list:
                    continue
                next_node=(neighbor[0],neighbor[1],curr[3]+1,self.getManhattanDistance(neighbor[0],end))
                parent[(next_node[0],next_node[1])]=(curr[0],curr[1])
                open_list.put(next_node,next_node[3])
        # print(path)
        return path


    def getManhattanDistance(self,loc1:int,loc2:int)->int:
        loc1_x=loc1//self.env.cols
        loc1_y=loc1%self.env.cols
        loc2_x=loc2//self.env.cols
        loc2_y=loc2%self.env.cols
        return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)


    def validateMove(self,loc:int,loc2:int)->bool:
        loc_x=loc//self.env.cols
        loc_y=loc%self.env.cols
        if(loc_x>=self.env.rows or loc_y>=self.env.cols or self.env.map[loc]==1):
            return False
        loc2_x=loc2//self.env.cols
        loc2_y=loc2%self.env.cols
        if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y)>1):
            return False
        return True

    def getNeighbors(self,location:int,direction:int):
        neighbors=[]
        candidates=[location+1,location-self.env.cols,location-1,location+self.env.cols]
        forward=candidates[direction]
        new_direction=direction
        if (forward>=0 and  forward < len(self.env.map) and self.validateMove(forward,location)):
            neighbors.append((forward,new_direction))
        new_direction = direction-1;
        if (new_direction == -1):
            new_direction = 3
        neighbors.append((location,new_direction))
    
        new_direction = direction+1;
        if (new_direction == 4):
            new_direction = 0
        neighbors.append((location,new_direction))
        # print("debug!!!!!!!", neighbors)
        return neighbors


        


if __name__=="__main__":
    test_planner=pyMAPFPlanner()
    print("done!")