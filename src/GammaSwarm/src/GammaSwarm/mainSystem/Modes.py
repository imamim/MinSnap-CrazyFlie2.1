#Rahman ve Rahim Olan Allah'ın Adıyla,Hamd O'na Mahsustur!
#Muvaffakiyetimiz Yalnızca O'na aittir!
from gammaSwarm import GammaSwarm
from MerkezcilClass import *
from UavClass import *
from Initializer import *
from copy import deepcopy
import rospy
import numpy as np


class Modes:

    def __init__(self,modeList):
        self.modeList = modeList
        self.modeListIndex = 0
        self.mode = list(modeList[self.modeListIndex].keys())[0]
        init_params = modeList[self.modeListIndex].get(self.mode)
        self.real_enabled = init_params.real_enabled 
        #TODO Inıtıal pozisyonu gelmedi ise (baglanamadik ise) durumun burada tespit edilmesi gerekir, sistemden bastan atip, gerekli parametreler duzenlenir!!
        initial_position = systemInitializer(init_params)
        #Haberleşme yapısı oluşturuldu,sistem kuruldu
        self.Swarm = GammaSwarm(init_params,initial_position)
        #merkezcil classlar oluşturuldu
        self.central_formation = FormationClass()
        self.trajectory_class = TrajGenerator()

        self.area_dimension = init_params.area_dimensions

        #Mode changed
        self.modeListIndex+=1
        self.mode = list(modeList[self.modeListIndex].keys())[0]
        



    #TODO Kodun basinda durum kontrolu (Virtual Structure,Active mi degil mi gibi seylerin kontrolu)
    def takeOffStep(self,takeoff_params):
        completed_list = []
        #TODO VStructure
        self.Swarm.update_SwarmCenter()
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.takeOffPID(uav.initial_position,takeoff_params.takeoff_height,self.Swarm.swarm_center,takeoff_params.threshold)
                #print(uav_result,uav.id,"target height: ",takeoff_height,"Current height: ",uav.current_position.x,uav.current_position.y,uav.current_position.z)
            else :
                uav_result = True

            completed_list.append(uav_result)
            uav.collisionAvoidance()
            uav.generateCommandMessage()
        self.Swarm.publishCommand()
        
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]        
        del completed_list










    #TODO Loiter Buglari analiz edilip düzeltilecek, zamansız loiter modu da eklenecek, eğer -1 ise time gibi veya başka şekilde çözülür.
    def loiterStep(self,loiter_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()

        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.loiter(loiter_params.loiter_time,self.Swarm.swarm_center)
            else :
                uav_result = True #Bundan dolayı sıkıntı oluyor!!!! Activation Flag Mevzusu

            completed_list.append(uav_result)
            uav.collisionAvoidance()
            uav.generateCommandMessage()
        self.Swarm.publishCommand()
        
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]
            for uav in self.Swarm.uav_list:
                uav.passing_loiter_time = 0 
                uav.loiter_active = False   
        del completed_list



    def formationStep2D(self,formation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()
        #TODO son noktamız, bir sonraki initial_pointsler buradan cikan outputlar olmalı Vstructure'de? Olmamalı da olabilir. Bunun outputu baslangic olmali
        #TODO aslında bu for donguleri scalable degil sistem icin. Ama her drone kendisi olursa cozulur gerceklemede!!!
        if self.central_formation.formation_created == False:
            self.central_formation.meanAnglePointCalculator(self.Swarm.uav_count,self.Swarm.uav_list,self.Swarm.swarm_center)
            self.central_formation.generateFormationPoints(self.Swarm.swarm_center,self.Swarm.uav_count,formation_params)
            #self.central_formation.FairMacar(self.Swarm.uav_list,self.Swarm.uav_count)
            self.central_formation.formation_created = True
        i = 0
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.goToPID(Position(self.central_formation.formation_points[self.central_formation.goal_indexes[i]][0],self.central_formation.formation_points[self.central_formation.goal_indexes[i]][1],self.central_formation.formation_points[self.central_formation.goal_indexes[i]][2]),formation_params.threshold,self.Swarm.swarm_center)
                i+=1
                #print(uav_result,uav.id,"target height: ",takeoff_height,"Current height: ",uav.current_position.x,uav.current_position.y,uav.current_position.z)
            else :
                uav_result = True
                i+=1
            completed_list.append(uav_result)
            uav.collisionAvoidance()
            uav.generateCommandMessage()
        self.Swarm.publishCommand()
        
        if all(completed_list) == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.central_formation.formation_created = False
            #TODO TUM PARAMETRELER SIFIRLANACK MERKEZCILCLASSTAKI        
        del completed_list





    def simpleNavigationStep(self,navigation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()

        #Bismillahirrahmanirrahim. Bizim buradaki amacımız navigation ile hedefe yaklaşabileceğimiz kadar yaklaşmak. Ardından eger hala varamadıysak.GoTo ile 
        #ihtiyac duyulan threshold degerine dronelar sokulur. GoTo mod olarak veyahut burada cagırılabılır! 
        #Mode baslangıcında trajectory uretilir ve bir daha buraya girmez!
        if self.trajectory_class.trajectory_generated == False:
            self.trajectory_class.generate_trajectory(self.Swarm.swarm_center,navigation_params.navigation_waypoints,navigation_params.max_velocity,navigation_params.agressiveness_kt)
            self.trajectory_class.trajectory_generated = True
            #TODO Trajectory Executer olusturuldugunda ilk stebi donse ve sonrasında steplense iyi olabilir ? veyahut timedan ayarlayacagım!
            self.trajectory_executer = TrajectoryExecuter(des_state = self.trajectory_class.get_des_state,Tmax = self.trajectory_class.TS[-1])
            #Simdi bunu kurduk her trajectory olusturulunca tekrar kurulacak ve steplenecek

        #Bismillahirrahmanirrahim
        desired_traj_state,traj_completed = self.trajectory_executer.step() #t = 0,0+1/56,0+2/56,......
        #Kontrol Stateleri:
        #1- Time'ın Tmax'ı geçmiş olması
        #2- Sürü merkezinin hedef noktasına olan uzaklığı
        #3- Veya her drone için

        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True and traj_completed == False:
                #Update Desired Setpoint
                uav.navigationEqualizer(desired_traj_state.desired_position,desired_traj_state.desired_velocity,desired_traj_state.desired_acceleration,desired_traj_state.desired_yaw,desired_traj_state.desired_omega)
                #Calculate actual error
                uav_result = uav.calculateNavigationError(self.trajectory_class.waypoints[-1],self.Swarm.swarm_center,navigation_params.threshold) #After navigation, if norm<Thrsehold uav_result become True
                #Not used
                uav.collisionAvoidance()
                #Then update message!
                uav.generateTrajectoryCommandMessage()
            #Inıtıal posıtıon her turlu degsiiyor    
            #Bunu kapayınca self.initial_position() guncellenmedigi icin sıkıntı yapıyor. Bence bu kalabilir
            elif uav.activation_flag == True and traj_completed == True:
                #EN GUNCEL TRAJ DURUMUNU DEPOLAMAK ICIN BUNUN CALISMASINDA BIR BEIS YOK. EN SON TRAJC POSITION'A GOTURUYOR! Trajc_completed olunca son point ve yaw degerı donuyor.
                uav.navigationEqualizer(desired_traj_state.desired_position,desired_traj_state.desired_velocity,desired_traj_state.desired_acceleration,desired_traj_state.desired_yaw,desired_traj_state.desired_omega)
                uav_result = uav.goToPID(uav.desired_trajectory_position,navigation_params.threshold,self.Swarm.swarm_center)
                uav.collisionAvoidance()
                uav.generateCommandMessage()
            else :
                uav_result = True

            completed_list.append(uav_result)
        if traj_completed == False:
            self.Swarm.publishTrajectoryCommand()
        elif traj_completed == True:
            self.Swarm.publishCommand()
        
        if all(completed_list) == True: #and: #traj_completed == True:
            self.Swarm.update_SwarmCenter() #-> Eger ki kabul edilebilir sınırlar icerisinde isek mod bitiminde swarm_center'ımızı guncelledik. No Problem
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.trajectory_class.trajectory_generated = False
            del self.trajectory_executer
            #TODO TUM PARAMETRELER SIFIRLANACK MERKEZCILCLASSTAKI        
        del completed_list











    #TODO If usage neede, pls go in the mission.py ---> change simpleNavigationStep() ---->individualNavigationStep()
    #Pls use only 1 drone when flight! Elhamdulillah navigation cozuldu
    """
    YAPILACAKLAR:
    1- TrajectoryExecuter adam edilecek, time mevzusu nasıl ayarlanır bakılacak. Olmadı el ile dt güzel bir degere konulacak.
    2- PID kontrolcu trajectory tracking'e konulacak. Ikı farklı kontrolcu olabilir.
    3- baska etkileyecek mevzular var ise bakılmalı. Loggingten traj_desired_position ve anlık pozisyon karsılastırılmalı
    """
    #This function for 1 Drones TESTING THE TRAJECTORY
    def individualNavigationStep(self,navigation_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()

        #Bismillahirrahmanirrahim. Bizim buradaki amacımız navigation ile hedefe yaklaşabileceğimiz kadar yaklaşmak. Ardından eger hala varamadıysak.GoTo ile 
        #ihtiyac duyulan threshold degerine dronelar sokulur. GoTo mod olarak veyahut burada cagırılabılır! 
        #Mode baslangıcında trajectory uretilir ve bir daha buraya girmez!
        if self.trajectory_class.trajectory_generated == False:
            #Hızlandırmalıyız bu generation'u
            self.trajectory_class.generate_trajectory(self.Swarm.uav_list[0].current_position,navigation_params.navigation_waypoints,navigation_params.max_velocity,navigation_params.agressiveness_kt)
            self.trajectory_class.trajectory_generated = True
            #TODO Trajectory Executer olusturuldugunda ilk stebi donse ve sonrasında steplense iyi olabilir ? veyahut timedan ayarlayacagım!
            self.trajectory_executer = TrajectoryExecuter(des_state = self.trajectory_class.get_des_state,Tmax = self.trajectory_class.TS[-1])
            #Simdi bunu kurduk her trajectory olusturulunca tekrar kurulacak ve steplenecek

        #Bismillahirrahmanirrahim
        desired_traj_state,traj_completed = self.trajectory_executer.step() #t = 0,0+1/56,0+2/56,......
        #Kontrol Stateleri:
        #1- Time'ın Tmax'ı geçmiş olması
        #2- Sürü merkezinin hedef noktasına olan uzaklığı
        #3- Veya her drone için

        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                #Update Desired Setpoint
                uav.individualNavigationEqualizer(desired_traj_state)
                #Calculate actual error
                uav_result = uav.individualCalculateNavigationError(self.trajectory_class.waypoints[-1],self.Swarm.swarm_center,navigation_params.threshold) #After navigation, if norm<Thrsehold uav_result become True
                #Not used
                uav.collisionAvoidance()
                #Then update message!
                uav.generateTrajectoryCommandMessage()
            else :
                uav_result = True

            completed_list.append(uav_result)

        self.Swarm.publishTrajectoryCommand()
        
        if traj_completed == True: #and: #traj_completed == True:
            print("BITTTTTTIIIII",self.Swarm.uav_list[0].current_position.x,self.Swarm.uav_list[0].current_position.y,self.Swarm.uav_list[0].current_position.z)
            self.Swarm.uav_list[0].initial_position = self.Swarm.uav_list[0].current_position
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0] 
            self.trajectory_class.trajectory_generated = False
            del self.trajectory_executer
            #TODO TUM PARAMETRELER SIFIRLANACK MERKEZCILCLASSTAKI        
        del completed_list










    def landingStep(self,landing_params):
        completed_list = []
        self.Swarm.update_SwarmCenter()
        for uav in self.Swarm.uav_list:
            if uav.activation_flag == True:
                uav_result = uav.landingPID(uav.initial_position,landing_params.threshold)
                #print(uav_result,uav.id,"target height: ",takeoff_height,"Current height: ",uav.current_position.x,uav.current_position.y,uav.current_position.z)
            else :
                uav_result = True
            completed_list.append(uav_result)
            uav.collisionAvoidance()
            uav.generateCommandMessage()
            #Sonra publish edilecek #uav.command_message()
        self.Swarm.publishCommand()
        
        if all(completed_list) == True:
            self.modeListIndex+=1
            self.mode = list(self.modeList[self.modeListIndex].keys())[0]         
        del completed_list

    