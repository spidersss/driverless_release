#include<fstream>
#include<iostream>
using namespace std;

ofstream fout;
int main()
{
	double path[3] = {1.333, 2.3, 3.4};
	fout.open("/home/wuconglei/route");
	for(int i = 0; i < 2; i++) fout<<path[i]<<"\t";
	fout<<"\n"<<path[2]<<"\n";
	fout.close();
	double a = 0;
	ifstream fin("/home/wuconglei/route");
	while(fin >> a) cout<<a<<"\t"; 
	fin.close();
	return 0;
}
