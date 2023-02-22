#!/usr/bin/env python
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import rospy
import os
from cyanob_phd_filter.msg import diagnostics
from datetime import datetime

class time_update_parallel:

    def __init__ (self,prefilePath,w_limit,_filename,_n,_f,_ftheta,_t):
        print("TUP->printing inside Parallel TU filter python ")
        #prefilePath - path to folder, ~ can use this 
        #w_limit-pruning weight
        print(prefilePath,w_limit,_filename,_n,_f,_ftheta,_t)


        print("TUP-> File path - ",prefilePath+_filename)
        self.initfile="initial_ground_truth.txt"
        df_initial_pts =pd.read_csv(os.path.expanduser(prefilePath+self.initfile),delim_whitespace=True,header=None,names=['x','y','w'])
        df_initial_pts.loc[:,'w']=0.5 #change all the weights equals to 0.5
        #print(df_initial_pts)


        self.mean =0 #mean for noise

        self.cov=_n   #covariance for the noise

        self.weightlowerlimit=w_limit #all point below given weight is deleted

        self.f=_f #forcing term magnitude
        self.ftheta=_ftheta #the angle of the wind
        self.t=_t #tempreture
        
        #self.ros_start()

        # Create target Directory if don't exist
        dirName=prefilePath+'/raw_camera_results'
        if not os.path.exists(dirName):
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ")
        else:    
            print("Directory " , dirName ,  " already exists")
        


        self.fpath=os.path.expanduser(prefilePath+_filename)
        try:
            self.readfile(self.fpath)
        except OSError: #os error will be raised inside the readfile func if file is unreadable
            print(self.fpath)
            print('TUP->file path error in time update....')
            #new_msg.time_update='file path error'
            #new_msg.measurement_update='filepath error'
            #_pub.publish(new_msg)
            exit()
        self.df=self.forcing_term(self.df,self.f,self.ftheta)
        self.df=self.temperature(self.df,self.t)
        print("2")
        self.resample()#normlizing weights. all points with weights >1 will be added again. w changed to 1
        self.df2=self.add_noise(self.df)#adding noise
        self.save_txt(os.path.expanduser(prefilePath+'WOIP'+_filename),self.df2)#saving tu copy without inital points
        self.df2=self.df2.append(df_initial_pts)#insert initial points to each TU.
        #print(self.df2)
        self.save_txt(self.fpath,self.df2)
        #return self.newpath
        #self.plot(self.df,self.df2)

    def get_path(self):
        return self.newpath

        
        # new_msg.time_update='ready'
        # new_msg.filename=self.newpath
        # new_msg.measurement_update= 'activate'
        # _pub.publish(new_msg)
        #del self

    @classmethod #allows class to be used without initiation.very usefull!!! check results analyzer
    def readfile (self,file_path):
        #if file not found, exit
        if os.path.isfile(file_path) == False:
            print("TUP->File not found..!!! ",file_path)
            raise OSError() # raise a exception so it can be catched in callback fun
            exit()
        
        self.df =pd .read_csv(file_path,delim_whitespace=True,header=None,names=['x','y','w'],skiprows=1,skipfooter=2)
        #print(self.df)
        if self.df.empty:# if file is empty, exit!!!
            print("TUP->empty file",file_path)
            exit()


    def resample (self):
        self.filter_weights(self.weightlowerlimit)
        self.df['w']=self.df['w'].apply(np.ceil) #rounding up the weights

        self.df_greaterthan1=self.df.loc[self.df.w>1]
        self.df.loc[self.df.w>1,'w']=1

        for idx in self.df_greaterthan1.index:
            #print(idx)
            #print(self.df.loc[idx],len(self.df.index))
            for i in range(int(self.df_greaterthan1.at[idx,'w'])-1):
                self.df.loc[len(self.df.index)]=self.df.loc[idx]


    def filter_weights(self,w_limit):
        self.df.drop(self.df[self.df['w']<w_limit].index,inplace = True)

    @classmethod
    def forcing_term(self,df,magnitude,angle): #angle - angle starting from North(degrees)
        xcomp=magnitude*np.sin(np.deg2rad(angle))
        ycomp=magnitude*np.cos(np.deg2rad(angle))
        
        return df+[xcomp, ycomp, 0]
    @classmethod
    def temperature(self,df,val): #value should be in celcius
        #25c above is optimal for growth googled
        mulfac=((val-25.0)/100)+1
        print (mulfac)
        return df*[1,1,mulfac]



    @classmethod 
    def plot(self,df_original ,df_processed):
        ax1=df_original.plot.scatter(x='x',y='y', color="DarkBlue", label="original")
        df_processed.plot.scatter(x='x',y='y', color="Orange", label="with noise",ax=ax1,alpha=0.7)
        #plt.show()
        plt.pause(1.00)
        #input("Press Enter to continue...")
        plt.close()
        

    def save_txt(self,path2save,data):
        #print('TUP->saving text')
        threeparts=path2save.rpartition("-") #find _ in the string and save in a three tuple
        #print(threeparts[0])
        if(len(threeparts[0])!=0):
            remove_mu=threeparts[0]
            remove_mu=remove_mu.replace('_mu','',1)
            #print(threeparts[2])
            #num=threeparts[2][:-4]
            self.newpath=remove_mu+threeparts[1]+str(int(threeparts[2][:-4])+1)+'.txt' #increment file by one number
        
        else:

            self.newpath=path2save.replace(".txt","-1.txt")

        print('TUP->saving file -',self.newpath)
        data.to_csv(path_or_buf=self.newpath,index=False, sep=" ")


    def add_noise(self, dataframe):


        original=dataframe.drop(["w"],axis=1).values#translating to numpy data
        #print(original)
        noise=np.random.normal(self.mean ,self.cov,size=(len(dataframe.axes[0]),2))
        noise=np.round(noise,decimals=2) #number of decimals to consider
        #print(noise)
        newpts=original+noise
        df_noise=pd.DataFrame(data=newpts,columns=['x','y'])# converting back to dataframe
        df_noise.round(decimals=3)

        df_noise.loc[df_noise.x<-150,'x']=-140 #-150.0 -100.0,-150.0 100.0,150.0 -100.0,150.0 100.0
        df_noise.loc[df_noise.x>150,'x']=140
        df_noise.loc[df_noise.y<-100,'y']=-90
        df_noise.loc[df_noise.y>100,'y']=90


        df_noise.insert(2,"w",dataframe["w"])
        #print(df_noise)

        return df_noise

    def isPtInside(self,dataframe):
        pass



    def __del__(self):#destructor for time update class
        print("TUP->time update destructor called")



# def callback_diagnostics(msg):
#     if msg.time_update== "activate" or msg.time_update=="request_status" or msg.time_update== "hold":

#         tu=time_update(msg,_prefilePath,0.1,_status)# (copy of diagnostic msg,folder path, min weight, status to run time update)

#         del tu #deleting the time update instance


# def main():

#     rospy.init_node('time_update_node',anonymous=True)

#     global _prefilePath

#     _prefilePath =rospy.get_param("folder_path",'~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/')
    
#     global _initfname #initial file name

#     _initfname=rospy.get_param("init_filename","initial_ground_truth.txt")

#     if os.path.isfile(os.path.expanduser(_prefilePath+_initfname)) == False:
#         print('file name or folder path is incorrect. check launch file!!!!!!!!!!!!!!')
#         exit()

#     f=open(os.path.expanduser(_prefilePath+'tu_constans'),"a")
#     now = datetime.now()
#     datestamp=now.strftime("%m/%d/%Y, %H:%M:%S")
#     f.write('\n')
#     f.write(datestamp)
#     f.write('\n')
#     f.write('================================\n')
#     f.write('noise covariace ='+str(cov))
#     f.close()
#     global df_initial_pts #reading initial points to a global data frame. Then add this to each time update.
#     df_initial_pts =pd .read_csv(os.path.expanduser(_prefilePath+_initfname),delim_whitespace=True,header=None,names=['x','y','w'])
#     df_initial_pts.loc[:,'w']=0.5 #change all the weights equals to 0.5
#     #print(df_initial_pts)

#     global _pub

#     _pub=rospy.Publisher('system_Diagnostics', diagnostics, queue_size=10, latch=True )

#     rospy.Subscriber("system_Diagnostics", diagnostics, callback_diagnostics)

#     global _status

#     _status="ready"

#     print("Time Update node ready...")
    
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down Time update node")


# if __name__== "__main__":
#     global cov
#     cov=14 #noise covariance
#     main()

#     global _prefilePath

#     #_prefilePath =rospy.get_param("folder_path",'~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/')
    
#     global _initfname #initial file name

#    # _initfname=rospy.get_param("init_filename","initial_ground_truth.txt")

#     if os.path.isfile(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/initial_ground_truth.txt')) == False:
#         print('file name or folder path is incorrect. check launch file!!!!!!!!!!!!!!')
#         exit()

#     global df_initial_pts #reading initial points to a global data frame. Then add this to each time update.
#     df_initial_pts =pd .read_csv(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/initial_ground_truth.txt'),delim_whitespace=True,header=None,names=['x','y','w'])
#     df_initial_pts.loc[:,'w']=0.5 #change all the weights equals to 0.5
#     #print(df_initial_pts)
    
#     ax1=df_initial_pts.plot.scatter(x='x',y='y', color="DarkBlue") # plotting 
#     df=time_update.forcing_term(df_initial_pts,90,350)
#     df.plot.scatter(x='x',y='y',ax=ax1, color="green",alpha=0.7) # plotting 
#     plt.show()

#     print(time_update.temperature(df,20))