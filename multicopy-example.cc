/*
 * geoflow-example.cc
 *
 * This test script creates a grid topology and then ping each other
 * switch WiFi[1.0.0.1]
 * switch WiFi[1.0.0.2] <-- channel --> controller
 * switch WiFi[1.0.0.3]
 *
 * Created on: 2013å¹´12æœˆ22æ—¥
 * Author: sumoon
 */

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

#include "ns3/simulator.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"

#include "ns3/geoflow-switch.h"
#include "ns3/geoflow-multicopy-routing.h"

using namespace ns3;

int carNum = 20;					// number of vehicles
int totalTime = 376;				// simulation time
int speed = 20;						// maximum of vehicular speed
int testNum = 10;					// number of test for each simulation
int sid = 0;						// id of source car
int did = 10;						// id of destination car
int copyNum = 2;					// number of copy
int routingType = 1;				// routing scheme used in the simulation
int qualityType = 1;				// quality type
double distWeight = 0.6;			// weight of distance to the destination
double angleWeight = 1-distWeight;	// weight of angle to existing copies
double range = 250;					// communication range
double minDist = 500;				// minimum distance from source to destination
double maxDist = 600;				// maximum distance from source to destination

NodeContainer switches;
ns3::geoflow::MulticopyRouting mcr;
std::ofstream out;
std::ifstream in;

void RouteMessage(){
	std::vector<ns3::geoflow::Switch> cars;
	std::vector <std::vector<bool> > con;

	//display vehicle position and copy state
	for (int i=0; i<carNum; i++)
	{
		ns3::geoflow::Switch s;
		s.SetId(i);
		Ptr<MobilityModel> mm = switches.Get(i)->GetObject<MobilityModel>();
		Vector pos = mm->GetPosition(); // Get position
		Vector vel = mm->GetVelocity(); // Get velocity
		s.SetPos(pos);
		s.SetVel(vel);
		cars.push_back(s);
		int hasCopy = 0;
		if (mcr.CheckCarHasCopy(i,mcr.getCopyCars())){
			hasCopy = 1;
		}
		out<<pos.x<<" "<<pos.y<<" "<<hasCopy<<std::endl;
	}

	//update topology of vehicles
	for (int i=0; i<carNum; i++){
		std::vector<bool> row;
		Ptr<MobilityModel> mm1 = switches.Get(i)->GetObject<MobilityModel>();
		Vector pos1 = mm1->GetPosition(); // Get position
		for (int j=0; j<carNum; j++){
			Ptr<MobilityModel> mm2 = switches.Get(j)->GetObject<MobilityModel>();
			Vector pos2 = mm2->GetPosition(); // Get position
			double dist = sqrt(pow(pos1.x-pos2.x,2)+pow(pos1.y-pos2.y,2));
			if (dist <= range){
				row.push_back(true);
			} else {
				row.push_back(false);
			}
		}
		con.push_back(row);
	}

	mcr.UpdateTopology(cars,con);

	//double elapseTime = Simulator::Now().GetSeconds();

	if (!mcr.isDelivered()){
		Time nextTime = MilliSeconds(200);
		Simulator::Schedule (nextTime, RouteMessage);
	}
}

int main (int argc, char **argv)
{

	CommandLine cmd;
	cmd.Parse (argc, argv);

	LogComponentEnable("GeoFlowMulticopyRouting", LOG_LEVEL_ALL);

	out.open("./scratch/trace.txt");

	//real hongkong large map

	std::vector<int> simTimeReal;
	simTimeReal.push_back(460);
	simTimeReal.push_back(471);
	simTimeReal.push_back(534);
	simTimeReal.push_back(696);

	carNum = 25;

	for (copyNum=2; copyNum<=5; copyNum+=1){
		//std::cout<<"	carNum is "<<carNum<<std::endl;
		totalTime = simTimeReal[(carNum-25)/25];
		std::vector<double> delayAvg;
		std::vector<double> deliverdNum;
		std::vector<double> forwardNum;
		for (routingType=0; routingType<=4; routingType++){
			delayAvg.push_back(0);
			deliverdNum.push_back(0);
			forwardNum.push_back(0);
		}

		for (testNum=1; testNum<=30; testNum++){
			sid = random()%carNum;
			did = random()%carNum;
			for (routingType=0; routingType<=4; routingType++){
				NodeContainer newSwitches;
				switches = newSwitches;
				ns3::geoflow::MulticopyRouting newMCR;
				mcr = newMCR;
				mcr.setSid(sid);
				mcr.setDid(did);
				mcr.setCopyNum(copyNum);
				mcr.setRoutingType(routingType);
				mcr.setQualityType(qualityType);
				mcr.setDistWeight(distWeight);
				mcr.setAngleWeight(angleWeight);

				switches.Create(carNum);

				std::stringstream ss;
				std::string traceFile;
				ss<<"./scratch/mobility-real-large-"<<carNum<<"-"<<totalTime<<".tcl";
				ss>>traceFile;
				ss.clear();
				Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
				ns2.Install ();

				RouteMessage();

				Simulator::Stop (Seconds (totalTime));
				Simulator::Run ();
				Simulator::Destroy ();

				if (mcr.isDelivered()){
					delayAvg[routingType] += mcr.getDeliverTime();
					deliverdNum[routingType]++;
				}
				forwardNum[routingType] += mcr.getForwardNum();
			}
		}

		for (routingType=0; routingType<=4; routingType++){
			delayAvg[routingType] = delayAvg[routingType]/deliverdNum[routingType];
			forwardNum[routingType] = forwardNum[routingType]/30;
			std::cout<<deliverdNum[routingType]<<" "<<delayAvg[routingType]<<" "<<forwardNum[routingType]<<" ";
		}
		std::cout<<std::endl;
	}

	//real hongkong small map

	/*std::vector<int> simTimeReal;
	simTimeReal.push_back(271);
	simTimeReal.push_back(397);
	simTimeReal.push_back(474);
	simTimeReal.push_back(478);
	simTimeReal.push_back(611);
	simTimeReal.push_back(697);
	simTimeReal.push_back(855);
	simTimeReal.push_back(856);
	simTimeReal.push_back(855);

	for (carNum=10; carNum<=40; carNum+=10){
		//std::cout<<"	carNum is "<<carNum<<std::endl;
		totalTime = simTimeReal[(carNum-10)/10];
		std::vector<double> delayAvg;
		std::vector<double> deliverdNum;
		for (routingType=0; routingType<=4; routingType++){
			delayAvg.push_back(0);
			deliverdNum.push_back(0);
		}

		for (testNum=1; testNum<=30; testNum++){
			sid = random()%carNum;
			did = random()%carNum;
			for (routingType=0; routingType<=4; routingType++){
				NodeContainer newSwitches;
				switches = newSwitches;
				ns3::geoflow::MulticopyRouting newMCR;
				mcr = newMCR;
				mcr.setSid(sid);
				mcr.setDid(did);
				mcr.setCopyNum(copyNum);
				mcr.setRoutingType(routingType);
				mcr.setQualityType(qualityType);
				mcr.setDistWeight(distWeight);
				mcr.setAngleWeight(angleWeight);

				switches.Create(carNum);

				std::stringstream ss;
				std::string traceFile;
				ss<<"./scratch/mobility-real-small-"<<carNum<<"-"<<totalTime<<".tcl";
				ss>>traceFile;
				ss.clear();
				Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
				ns2.Install ();

				RouteMessage();

				Simulator::Stop (Seconds (totalTime));
				Simulator::Run ();
				Simulator::Destroy ();

				if (mcr.isDelivered()){
					delayAvg[routingType] += mcr.getDeliverTime();
					deliverdNum[routingType]++;
				}
			}
		}

		for (routingType=0; routingType<=4; routingType++){
			delayAvg[routingType] = delayAvg[routingType]/deliverdNum[routingType];
			std::cout<<deliverdNum[routingType]<<" "<<delayAvg[routingType]<<" ";
		}
		std::cout<<std::endl;
	}*/

/*
	carNum = 51;
	totalTime = 50;
	copyNum = 1;
	routingType = 2;
	sid = 0;
	did = 2;

	NodeContainer newSwitches;
	switches = newSwitches;
	ns3::geoflow::MulticopyRouting newMCR;
	mcr = newMCR;
	mcr.setSid(sid);
	mcr.setDid(did);
	mcr.setCopyNum(copyNum);
	mcr.setRoutingType(routingType);
	mcr.setQualityType(qualityType);
	mcr.setDistWeight(distWeight);
	mcr.setAngleWeight(angleWeight);

	switches.Create(carNum);

	std::stringstream ss;
	std::string traceFile;
	ss<<"./scratch/taxi-mobility.tcl";
	ss>>traceFile;
	ss.clear();
	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install ();

	RouteMessage();

	Simulator::Stop (Seconds (totalTime));
	Simulator::Run ();
	Simulator::Destroy ();

	out.close();

	if (mcr.isDelivered()){
		std::vector<Forward> fwVec = mcr.getTrace().at(did);
		Forward fw;
		fw.setFromId(did);
		fw.setForwardTime(mcr.getDeliverTime());
		fwVec.push_back(fw);
		std::vector<Forward>::size_type pos = 0;
		fw = fwVec.at(pos);
		int copyID = fw.getFromId();
		double changeTime = fw.getForwardTime();

		in.open("./scratch/trace.txt");
		out.open("./scratch/trace1.txt");

		double runTime = 0;
		double carCount = 0;
		double posx;
		double posy;
		int hasCopy;
		while (!in.eof() )
		{
			in>>posx;
			in>>posy;
			in>>hasCopy;
			if (carCount!=copyID){
				hasCopy = 0;
			}
			out<<posx<<" "<<posy<<" "<<hasCopy<<std::endl;

			carCount++;
			if (carCount == carNum){
				runTime = runTime + 0.2;
				carCount = 0;
			}
			if (runTime > changeTime && changeTime < mcr.getDeliverTime()){
				pos++;
				fw = fwVec.at(pos);
				copyID = fw.getFromId();
				changeTime = fw.getForwardTime();
			}
		}

		in.close();
		out.close();
	}
*/

/*
	speed = 20;
	carNum = 20;
    totalTime = 124;
    copyNum = 1;
    routingType = 0;
    for (int k=0;k<100;k++){
    	routingType = 0;
		sid = random()%carNum;
		did = random()%carNum;
		std::cout<<"sid = "<<sid<<" did = "<<did<<std::endl;

		NodeContainer newSwitches;
		switches = newSwitches;
		ns3::geoflow::MulticopyRouting newMCR;
		mcr = newMCR;
		mcr.setSid(sid);
		mcr.setDid(did);
		mcr.setCopyNum(copyNum);
		mcr.setRoutingType(routingType);
		mcr.setQualityType(qualityType);
		mcr.setDistWeight(distWeight);
		mcr.setAngleWeight(angleWeight);

		switches.Create(carNum);

		std::stringstream ss;
		std::string traceFile;
		ss<<"./scratch/mobility-grid-small-"<<speed<<"-"<<carNum<<"-"<<totalTime<<".tcl";
		ss>>traceFile;
		ss.clear();
		Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
		ns2.Install ();

		RouteMessage();

		Simulator::Stop (Seconds (totalTime));
		Simulator::Run ();
		Simulator::Destroy ();

		out.close();

		std::cout<<mcr.getDeliverTime()<<" ";

		routingType = 2;

		NodeContainer new1Switches;
		switches = new1Switches;
		ns3::geoflow::MulticopyRouting new1MCR;
		mcr = new1MCR;
		mcr.setSid(sid);
		mcr.setDid(did);
		mcr.setCopyNum(copyNum);
		mcr.setRoutingType(routingType);
		mcr.setQualityType(qualityType);
		mcr.setDistWeight(distWeight);
		mcr.setAngleWeight(angleWeight);

		switches.Create(carNum);



		ss<<"./scratch/mobility-grid-small-"<<speed<<"-"<<carNum<<"-"<<totalTime<<".tcl";
		ss>>traceFile;
		ss.clear();
		ns2 = Ns2MobilityHelper (traceFile);
		ns2.Install ();

		RouteMessage();

		Simulator::Stop (Seconds (totalTime));
		Simulator::Run ();
		Simulator::Destroy ();

		out.close();

		std::cout<<mcr.getDeliverTime()<<std::endl;

    }
*/


/*
	minDist = 500;
	maxDist = 600;

	std::vector<int> timeTemp;
	timeTemp.push_back(517);
	timeTemp.push_back(228);
	timeTemp.push_back(145);
	timeTemp.push_back(124);

	for (speed=5; speed<=20; speed+=5){
		std::cout<<"speed is "<<speed<<std::endl;
		carNum = 20;
		totalTime = timeTemp[(speed-5)/5];
		std::vector<double> delayAvg;
		std::vector<double> deliverdNum;
		std::vector<double> storeCost;
		for (routingType=0; routingType<=8; routingType++){
			delayAvg.push_back(0);
			deliverdNum.push_back(0);
			storeCost.push_back(0);
		}

		for (testNum=1; testNum<=10; testNum++){
			sid = random()%carNum;
			did = random()%carNum;
			for (routingType=0; routingType<=8; routingType++){
				NodeContainer newSwitches;
				switches = newSwitches;
				ns3::geoflow::MulticopyRouting newMCR;
				mcr = newMCR;
				mcr.setSid(sid);
				mcr.setDid(did);
				mcr.setCopyNum(copyNum);
				mcr.setRoutingType(routingType);
				mcr.setQualityType(qualityType);
				mcr.setDistWeight(distWeight);
				mcr.setAngleWeight(angleWeight);

				switches.Create(carNum);

				std::stringstream ss;
				std::string traceFile;
				ss<<"./scratch/mobility-grid-small-"<<speed<<"-"<<carNum<<"-"<<totalTime<<".tcl";
				ss>>traceFile;
				ss.clear();
				Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
				ns2.Install ();

				while(true){
					Ptr<MobilityModel> smm = switches.Get(sid)->GetObject<MobilityModel>();
					Vector spos = smm->GetPosition(); // Get position
					Ptr<MobilityModel> dmm = switches.Get(did)->GetObject<MobilityModel>();
					Vector dpos = dmm->GetPosition(); // Get position
					double dist = sqrt(pow((spos.x-dpos.x),2)+pow((spos.y-dpos.y),2));
					//std::cout<<"sid = "<<sid<<" did = "<<did<<" dist = "<<dist<<" minDist = "<<minDist<<std::endl;
					if(dist>=minDist && dist<=maxDist){
						break;
					} else {
						sid = random()%carNum;
						did = random()%carNum;
					}
				}
				//std::cout<<"sid = "<<sid<<" did = "<<did<<std::endl;

				RouteMessage();

				Simulator::Stop (Seconds (totalTime));
				Simulator::Run ();
				Simulator::Destroy ();

				if (mcr.isDelivered()){
					delayAvg[routingType] += mcr.getDeliverTime();
					deliverdNum[routingType]++;
					storeCost[routingType] += mcr.getStoreCost();
					//std::cout<<storeCost[routingType]<<std::endl;
				}
			}
		}

		for (routingType=0; routingType<=8; routingType++){
			delayAvg[routingType] = delayAvg[routingType]/deliverdNum[routingType];
			std::cout<<deliverdNum[routingType]<<" "<<delayAvg[routingType]<<" "<<storeCost[routingType]<<std::endl;
		}
	}
*/

	//grid small
/*
	minDist = 500;
	std::vector< std::vector<int> > simTimeSmall;
	std::vector<int> timeTemp;
	timeTemp.push_back(451);
	timeTemp.push_back(356);
	timeTemp.push_back(517);
	timeTemp.push_back(379);
	timeTemp.push_back(427);
	simTimeSmall.push_back(timeTemp);

	timeTemp.clear();
	timeTemp.push_back(156);
	timeTemp.push_back(190);
	timeTemp.push_back(228);
	timeTemp.push_back(226);
	timeTemp.push_back(207);
	simTimeSmall.push_back(timeTemp);

	timeTemp.clear();
	timeTemp.push_back(150);
	timeTemp.push_back(136);
	timeTemp.push_back(145);
	timeTemp.push_back(158);
	timeTemp.push_back(145);
	simTimeSmall.push_back(timeTemp);

	timeTemp.clear();
	timeTemp.push_back(144);
	timeTemp.push_back(138);
	timeTemp.push_back(124);
	timeTemp.push_back(124);
	timeTemp.push_back(133);
	simTimeSmall.push_back(timeTemp);

	for (speed=5; speed<=20; speed+=5){
		std::cout<<"speed is "<<speed<<std::endl;
		for (carNum=10; carNum<=30; carNum+=5){
			std::cout<<"	carNum is "<<carNum<<std::endl;
			totalTime = simTimeSmall[(speed-5)/5][(carNum-10)/5];
			std::vector<double> delayAvg;
			std::vector<double> deliverdNum;
			for (routingType=0; routingType<=8; routingType++){
				delayAvg.push_back(0);
				deliverdNum.push_back(0);
			}

			for (testNum=1; testNum<=30; testNum++){
				sid = random()%carNum;
				did = random()%carNum;
				for (routingType=0; routingType<=8; routingType++){
					NodeContainer newSwitches;
					switches = newSwitches;
					ns3::geoflow::MulticopyRouting newMCR;
					mcr = newMCR;
					mcr.setSid(sid);
					mcr.setDid(did);
					mcr.setCopyNum(copyNum);
					mcr.setRoutingType(routingType);
					mcr.setQualityType(qualityType);
					mcr.setDistWeight(distWeight);
					mcr.setAngleWeight(angleWeight);

					switches.Create(carNum);

					std::stringstream ss;
					std::string traceFile;
					ss<<"./scratch/mobility-grid-small-"<<speed<<"-"<<carNum<<"-"<<totalTime<<".tcl";
					ss>>traceFile;
					ss.clear();
					Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
					ns2.Install ();

					while(true){
						Ptr<MobilityModel> smm = switches.Get(sid)->GetObject<MobilityModel>();
						Vector spos = smm->GetPosition(); // Get position
						Ptr<MobilityModel> dmm = switches.Get(did)->GetObject<MobilityModel>();
						Vector dpos = dmm->GetPosition(); // Get position
						double dist = sqrt(pow((spos.x-dpos.x),2)+pow((spos.y-dpos.y),2));
						//std::cout<<"sid = "<<sid<<" did = "<<did<<" dist = "<<dist<<" minDist = "<<minDist<<std::endl;
						if(dist>=minDist){
							break;
						} else {
							sid = random()%carNum;
							did = random()%carNum;
						}
					}
					//std::cout<<"sid = "<<sid<<" did = "<<did<<std::endl;

					RouteMessage();

					Simulator::Stop (Seconds (totalTime));
					Simulator::Run ();
					Simulator::Destroy ();

					if (mcr.isDelivered()){
						delayAvg[routingType] += mcr.getDeliverTime();
						deliverdNum[routingType]++;
					}
				}
			}

			for (routingType=0; routingType<=8; routingType++){
				delayAvg[routingType] = delayAvg[routingType]/deliverdNum[routingType];
				std::cout<<deliverdNum[routingType]<<" "<<delayAvg[routingType]<<std::endl;
			}
		}
	}
*/

/*
	// grid large
	minDist = 1500;
	std::vector< std::vector<int> > simTimeLarge;
	std::vector<int> timeTemp;
	timeTemp.push_back(343);
	timeTemp.push_back(372);
	timeTemp.push_back(434);
	timeTemp.push_back(537);
	simTimeLarge.push_back(timeTemp);

	for (carNum=160; carNum<=160; carNum+=60){
		std::cout<<"	carNum is "<<carNum<<std::endl;
		totalTime = simTimeLarge[0][(carNum-40)/60];
		std::vector<double> delayAvg;
		std::vector<double> deliverdNum;
		for (routingType=0; routingType<=8; òÅüÿrÆüÿÇüÿ’Çüÿ(Æüÿ¸ÆüÿHÇüÿØÇüÿèÅüÿhÆüÿøÆüÿˆÇüÿÆüÿ®Æüÿ>ÇüÿÎÇüÿÞÅüÿ^ÆüÿîÆüÿ~ÇüÿÆüÿ¤Æüÿ4ÇüÿÄÇüÿÔÅüÿTÆüÿäÆüÿtÇüÿ
ÆüÿšÆüÿ*ÇüÿºÇüÿÊÅüÿJÆüÿÚÆüÿjÇüÿ ÆüÿÆüÿ Çüÿ°ÇüÿÀÅüÿ@ÆüÿÐÆüÿ`ÇüÿrÓüÿÔüÿ¢Ôüÿ2Õüÿ2Óüÿ€ÓüÿRÔüÿâÔüÿhÓüÿÔüÿ˜Ôüÿ(Õüÿ(Óüÿ¸ÓüÿHÔüÿØÔüÿ^ÓüÿþÓüÿŽÔüÿÕüÿÓüÿ®Óüÿ>ÔüÿÎÔüÿTÓüÿôÓüÿ„ÔüÿÕüÿÓüÿ¤Óüÿ4ÔüÿÄÔüÿJÓüÿêÓüÿzÔüÿ
Õüÿ
ÓüÿšÓüÿ*ÔüÿºÔüÿ@ÓüÿàÓüÿpÔüÿ Õüÿ ÓüÿÓüÿ Ôüÿ°Ôüÿ®ÔüÿnÕüÿÖüÿÞÖüÿN×üÿàÔüÿ¾ÕüÿnÖüÿ¤ÔüÿdÕüÿÖüÿÔÖüÿD×üÿÕüÿ´ÕüÿdÖüÿÊÔüÿŠÕüÿ:Öüÿ
×üÿj×üÿ*ÕüÿÚÕüÿšÖüÿšÔüÿZÕüÿ
ÖüÿÊÖüÿ:×üÿúÔüÿªÕüÿZÖüÿÀÔüÿ€Õüÿ0Öüÿ ×üÿ`×üÿ ÕüÿÐÕüÿÖüÿÔüÿPÕüÿ ÖüÿÀÖüÿ0×üÿðÔüÿ ÕüÿPÖüÿ`ÆüÿÉüÿ Éüÿ0ÊüÿÀÊüÿPËüÿàËüÿpÌüÿ ÍüÿÍüÿ Îüÿ°Îüÿ@ÏüÿÐÏüÿ`ÐüÿðÐüÿ¦×üÿ8Øüÿ"ÙüÿÒÙüÿ¢×üÿ2ØüÿÒØüÿrÙüÿè×üÿxØüÿÙüÿÈÙüÿ˜×üÿ(ØüÿÈØüÿhÙüÿÞ×üÿnØüÿÙüÿ¾ÙüÿŽ×üÿØüÿ¾Øüÿ^ÙüÿÔ×üÿdØüÿÙüÿ´Ùüÿ„×üÿØüÿ´ØüÿTÙüÿÊ×üÿZØüÿúØüÿªÙüÿz×üÿ
ØüÿªØüÿJÙüÿÀ×üÿPØüÿðØüÿ Ùüÿp×üÿ Øüÿ Øüÿ@Ùüÿråüÿ"æüÿ²æüÿBçüÿ2åüÿ€åüÿbæüÿòæüÿhåüÿæüÿ¨æüÿ8çüÿ(åüÿÈåüÿXæüÿèæüÿ^åüÿæüÿžæüÿ.çüÿåüÿ¾åüÿNæüÿÞæüÿTåüÿæüÿ”æüÿ$çüÿåüÿ´åüÿDæüÿÔæüÿJåüÿúåüÿŠæüÿçüÿ
åüÿªåüÿ:æüÿÊæüÿ@åüÿðåüÿ€æüÿçüÿ åüÿ åüÿ0æüÿÀæüÿ¾æüÿžçüÿnèüÿ>éüÿ®éüÿðæüÿþçüÿÎèüÿ´æüÿ”çüÿdèüÿ4éüÿ¤éüÿ$çüÿôçüÿÄèüÿÚæüÿºçüÿŠèüÿjéüÿÊéüÿZçüÿ*èüÿúèüÿªæüÿŠçüÿZèüÿ*éüÿšéüÿçüÿêçüÿºèüÿÐæüÿ°çüÿ€èüÿ`éüÿÀéüÿPçüÿ èüÿðèüÿ æüÿ€çüÿPèüÿ éüÿéüÿçüÿàçüÿ°èüÿ`ØüÿÛüÿ Ûüÿ0ÜüÿÀÜüÿPÝüÿàÝüÿpÞüÿ ßüÿßüÿ àüÿ°àüÿ@áüÿÐáüÿ`âüÿðâüÿêüÿˆêüÿbëüÿòëüÿêüÿ‚êüÿëüÿ¢ëüÿ8êüÿÈêüÿXëüÿèëüÿøéüÿxêüÿëüÿ˜ëüÿ.êüÿ¾êüÿNëüÿÞëüÿîéüÿnêüÿþêüÿŽëüÿ$êüÿ´êüÿDëüÿÔëüÿäéüÿdêüÿôêüÿ„ëüÿêüÿªêüÿ:ëüÿÊëüÿÚéüÿZêüÿêêüÿzëüÿêüÿ êüÿ0ëüÿÀëüÿÐéüÿPêüÿàêüÿpëüÿ¢øüÿùüÿbùüÿ²ùüÿœøüÿ°øüÿ\ùüÿ¬ùüÿ–øüÿùüÿVùüÿ¦ùüÿøüÿ ùüÿPùüÿ ùüÿŠøüÿúøüÿJùüÿšùüÿ„øüÿôøüÿDùüÿ”ùüÿ~øüÿîøüÿ>ùüÿŽùüÿxøüÿèøüÿ8ùüÿˆùüÿrøüÿâøüÿ2ùüÿ‚ùüÿløüÿÜøüÿ,ùüÿ|ùüÿføüÿÖøüÿ&ùüÿvùüÿ`øüÿÐøüÿ ùüÿpùüÿpìüÿ0ïüÿÀïüÿPðüÿàðüÿpñüÿ òüÿòüÿ óüÿ°óüÿ@ôüÿÐôüÿ`õüÿðõüÿ€öüÿ÷üÿ`úüÿ²úüÿûüÿRûüÿ\úüÿ¬úüÿüúüÿLûüÿVúüÿ¦úüÿöúüÿFûüÿPúüÿ úüÿðúüÿ@ûüÿJúüÿšúüÿêúüÿ:ûüÿDúüÿ”úüÿäúüÿ4ûüÿ>úüÿŽúüÿÞúüÿ.ûüÿ8úüÿˆúüÿØúüÿ(ûüÿ2úüÿ‚úüÿÒúüÿ"ûüÿ,úüÿ|úüÿÌúüÿûüÿ&úüÿvúüÿÆúüÿûüÿ úüÿpúüÿÀúüÿûüÿÂýÿ"ýÿrýÿÂýÿ¼ýÿÐýÿlýÿ¼ýÿ¶ýÿýÿfýÿ¶ýÿ°ýÿýÿ`