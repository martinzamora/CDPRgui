#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QTextStream>
#include <QString>
#include <stdio.h>
#include<stdlib.h>
#include <iostream>
#include <string>
#include<QProcess>
#include <fstream>
#include<QMediaPlayer>
#include<QVideoWidget>
#include<QMediaPlaylist>
#include<QSerialPort>
#include<QSerialPortInfo>
#include<windows.h>


#define stepSize 0.000475
#define COM_PORT "COM8"
#define baud_rate 12000000


int runs=0;
bool simulated=true;
std::vector <std::vector <float>> cableLenMat;
std::vector <std::vector <float>> stepMat;
using namespace std;

void writePoint(float a, float b, float c);
void stepCalc();
void serialCom();
void packetizer();

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_confirmbutton_clicked()
{

    int x = ui->lineEdit->text().toFloat();
    int y = ui->lineEdit_2->text().toFloat();
    int z = ui->lineEdit_3->text().toFloat();
    
    //change he file output location
    writePoint(x,y,z);

    std::cout<<"button"<<endl;
}
void MainWindow::on_confirmbutton_2_pressed(){
    QString closePoints = "\n\t\t\t</points>";
    QString closeP = "\n\t\t</quintic_spline_trajectory>\n\t</joint_trajectories>\n</trajectories>";
    //replace with your directory for the xml file
    QFile outputFile("C:\\Users\\Martin\\Downloads\\CASPR-master\\CASPR-master\\data\\model_config\\models\\SCDM\\spatial_manipulators\\PoCaBot_spatial\\PoCaBot_spatial_trajectories.xml");
    outputFile.open((QIODevice::WriteOnly | QIODevice::Append));
    QTextStream out(&outputFile);
    out<<closePoints << closeP;
    outputFile.close();
    return;
}

void writePoint(float a, float b, float c) {
    a=a/100;
    b=b/100;
    c=c/100;
    QString point = "\n\t\t\t\t<point>";
    QString closePoint = "\n\t\t\t\t</point>";
    QString qDot = "\n\t\t\t\t\t<q_dot>0.0 0.0 0.0 0.0 0.0 0.0</q_dot>\n\t\t\t\t\t<q_ddot>0.0 0.0 0.0 0.0 0.0 0.0</q_ddot>";
    QString coor = "\n\t\t\t\t\t<q>" + QString::number(a) + " " + QString::number(b) + " " + QString::number(c)+" 0.0 0.0 0.0</q>";
    //write point, coor+qDot and then closePoint
    //replace with your directory for the xml file
    QFile outputFile("C:\\Users\\Martin\\Downloads\\CASPR-master\\CASPR-master\\data\\model_config\\models\\SCDM\\spatial_manipulators\\PoCaBot_spatial\\PoCaBot_spatial_trajectories.xml");

    if(runs==0){
        QString head="<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<!DOCTYPE trajectories SYSTEM \"../../../../templates/trajectories.dtd\">";
        QString open = "\n<trajectories>\n\t<joint_trajectories>";
        QString openP = "\n\t\t<quintic_spline_trajectory id=\"traj_1\" time_definition = \"relative\" time_step=\"0.05\">";
        QString points = "\n\t\t\t<points>";
        outputFile.open((QIODevice::WriteOnly | QIODevice::Text));
        QTextStream out(&outputFile);
        out<<head+open+openP+points;
        out<<point << coor << qDot << closePoint;
        outputFile.close();
        runs++;
    }
    else{
        float time= runs; //control speed of cable robot with constant multiplied. inverse relationship between constant and speed
        QString point = "\n\t\t\t\t<point time=\""+ QString::number(time)+ "\">";
        outputFile.open((QIODevice::WriteOnly | QIODevice::Append));
        QTextStream out(&outputFile);
        out<<point << coor << qDot << closePoint;
        outputFile.close();
        runs++;
    }

    return;
}

void MainWindow::on_confirmbutton_3_clicked()
{
    //write a bat file to call matlab scripts and fix directory
    system("start C:\\Users\\Martin\\Desktop\\script.bat");
    simulated=true;
    return;
}

void MainWindow::on_confirmbutton_4_clicked()
{
    if(simulated==true){
        QMediaPlayer* player = new QMediaPlayer;
        QVideoWidget* vw= new QVideoWidget;
        player->setVideoOutput(vw);
        QMediaPlaylist* playlist =new QMediaPlaylist();
        //match directory with the directory where video is saved
        playlist->addMedia(QUrl::fromLocalFile("C:/Users/Martin/Downloads/CASPR-master/CASPR-master/data/videos/kinematics_gui_output.avi"));
        playlist->setPlaybackMode(QMediaPlaylist::Loop);
        vw->setGeometry(100,100,300,400);
        vw->show();
        player->setPlaylist(playlist);
        player->play();
    }
    else{
        cout<<"simulation not ran yet."<<endl;
    }
}

void MainWindow::on_confirmbutton_5_clicked()
{
    //match directory
    ifstream infile("C:\\Users\\Martin\\Desktop\\array.txt");
    string line;
    string delimiter= ",";
    string subvalue;
    size_t pos = 0;
    vector<float> temp;
    while(getline(infile, line)){
        while ((pos = line.find(delimiter)) != string::npos){
            subvalue=line.substr(0, line.find(delimiter));
            //cout<<subvalue<<endl;
            temp.push_back(atof(subvalue.c_str()));
            line.erase(0, pos + delimiter.length());
        }
        cableLenMat.push_back(temp);
        temp.clear();
    }
    //cableLenMat has vectors of floats. Each vector is a cable.
    //Run conversion of delta(cablelengths) to steps.
    //Store in stepMat.
    stepCalc();
    return;

}
void stepCalc(){
    int row,rowLen,col,steps;
    float delta;
    int nDel;
    std::vector <float> temp;
    std::vector <float> stepTemp;
    for(row=0; row<8; row++){
        temp=cableLenMat[row];
        rowLen=temp.size();
        for(col=0; col<rowLen-1; col++){
            nDel=temp[col+1]/stepSize;
            delta=temp[col+1]-temp[col];
            steps=delta/stepSize;
            stepTemp.push_back(steps);
            cout<<steps<<" "<<delta <<" "<<nDel <<endl;
        }
        stepMat.push_back(stepTemp);
        stepTemp.clear();
        temp.clear();
    }
    serialCom();
    return;
}
void serialCom(){
    QSerialPort port;
    port.setPortName(COM_PORT);
    port.setBaudRate(baud_rate);
    port.setParity(QSerialPort::NoParity);
    port.setStopBits(QSerialPort::OneStop);
    port.setFlowControl(QSerialPort::NoFlowControl);
    port.open(QIODevice::ReadWrite);
    cout<<"port opened"<<endl;
    QByteArray buffer = QByteArray::number(23) + "\n";
    port.write(buffer);
    port.flush();
    return;
}
void packetizer(){
    return;
}
