#include"ElecTowerReconstruction.h"


int main() {

	//���ݵ������в�Ҫ���ֿո�

	string filepath = "Input/JT2003.xyz";
	string out = "result/JT2003.obj";
	ElecTowerReconsrutction ss;
	ss.SetOutPath(out);
	ss.SetTowerType(1);
	ss.TopReconstrution(filepath);

}