#include"ElecTowerReconstruction.h"


int main() {

	//数据的名称中不要出现空格

	string filepath = "Input/JT2003.xyz";
	string out = "result/JT2003.obj";
	ElecTowerReconsrutction ss;
	ss.SetOutPath(out);
	ss.SetTowerType(1);
	ss.TopReconstrution(filepath);

}