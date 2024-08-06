#include "controlhelper.h"
/**
 * @brief 
 * Open the file to read float data to dest.    
 * In the file, ',' must be used after every data, including the last data.  
 * @param add The address of the file to read, like "../include/init_Motor_angle.csv"
 * @param dest Floating pointer to store datas.
 */
void string2float(const std::string& add, std::vector<float>& dest) {
    std::ifstream inidata(add);
    if (inidata) {
        std::cout << add << " file open Successful" << std::endl;
    } else {
        std::cerr << add << " file open FAIL" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(inidata, line)) {
        size_t start = 0, end = 0;
        while ((end = line.find(',', start)) != std::string::npos) {
            std::string token = line.substr(start, end - start);
            try {
                dest.push_back(std::stof(token));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid argument: " << e.what() << " for token: " << token << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range error: " << e.what() << " for token: " << token << std::endl;
            }
            start = end + 1;
        }
        // Process the last token if there's no trailing comma
        std::string token = line.substr(start);
        try {
            dest.push_back(std::stof(token));
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: " << e.what() << " for token: " << token << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Out of range error: " << e.what() << " for token: " << token << std::endl;
        }
    }

    inidata.close();
}

void string2float2(std::string add, float* dest)
{
    char data_char[8000],*char_float;
    const char *a=",";  //Separate datas
    int i=0;
    ifstream inidata;
    inidata.open(add);
    if (inidata)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    inidata.read(data_char,8000);
    char_float=strtok(data_char, a);
    while(char_float!=NULL)
    {        
        dest[i++] = stof(char_float);
        //cout<<'|'<<dest[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}

void printSvStatus(unsigned char svStatus)
{   
    std::cout<<"svStatus="<<!!((svStatus<<0)&0b10000000);
    for(int i=1;i<8;i++)
        std::cout<<!!((svStatus<<i)&0b10000000);
    std::cout<<std::endl;
}

Matrix<float, 4, 4> inverseMotorMapping(vector<float> motorPos)
{
    Matrix<float,4,4> jointPos;
    jointPos(0,0) = (motorPos[0]-motorPos[1])/2.0;
    jointPos(0,1) = (motorPos[0]+motorPos[1])/2.0;
    jointPos(1,0) = (motorPos[3]-motorPos[4])/2.0;
    jointPos(1,1) = (motorPos[3]+motorPos[4])/2.0;
    jointPos(2,0) = (motorPos[7]-motorPos[6])/2.0;
    jointPos(2,1) = (motorPos[7]+motorPos[6])/2.0;
    jointPos(3,0) = (motorPos[10]-motorPos[9])/2.0;
    jointPos(3,1) = (motorPos[10]+motorPos[9])/2.0;
    for(int i=0;i<4;i++)
    jointPos(i,2)= motorPos[i*3+2];

     for(int i=0;i<4;i++)
    jointPos(i,3)= motorPos[i+12];

    return jointPos;
}



vector<float> motorMapping(Matrix<float, 4, 3> jointCmdPos)
{
    vector<float> motorPos;
    for(int i=0;i<12;++i)
        motorPos.push_back(0);
    motorPos[0]=jointCmdPos(0,1)+jointCmdPos(0,0);
    motorPos[1]=jointCmdPos(0,1)-jointCmdPos(0,0);
    motorPos[3]=jointCmdPos(1,1)+jointCmdPos(1,0);
    motorPos[4]=jointCmdPos(1,1)-jointCmdPos(1,0);
    motorPos[6]=jointCmdPos(2,1)-jointCmdPos(2,0);
    motorPos[7]=jointCmdPos(2,1)+jointCmdPos(2,0);
    motorPos[9]=jointCmdPos(3,1)-jointCmdPos(3,0);
    motorPos[10]=jointCmdPos(3,1)+jointCmdPos(3,0);
    for(int i=0;i<4;i++)
    motorPos[i*3+2]=jointCmdPos(i,2);
   
    return motorPos;
};



int match(char *P, char *T)
{
    int *next=buildNext(P);
     int n=(int) strlen(T),i=0;
     int m=(int) strlen(P),j=0;
     while(j<m&&i<n)
     if(0>j||T[i]==P[j]){
        i++;j++;
     }else
        j=next[j];
    delete[] next;
    return i-j;
}

int *buildNext(char *P)
{
    size_t m=strlen(P),j=0;
    int *N=new int[m];
    int t=N[0]=-1;
    while(j<m-1)
        if(0>t||P[j]==P[t]){
            j++,t++;N[j]=P[j]!=P[t]?t:N[t];
        }else
            t=N[t];
    return N;
}

bool commandJudge(char *P, char *T)
{
    int ret=match(P,T);
    if(ret>=strlen(T)) return false;
    else if(ret<0){cout<<"match error"<<endl; return false;}
    else return true; 
}

void SetPos(Matrix<float,4,3> jointCmdPos,DxlAPI& motors,vector<float>& vLastSetPos)
{
    vector<float> setPos;
    setPos=motorMapping(jointCmdPos);
    setPos.resize(16);
    for(int i = 0;i< 4 ;i++)
    {
        if(i<2)
         setPos[i+12]=-jointCmdPos(i,0);
        else
         setPos[i+12]=jointCmdPos(i,0);
    }
   
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            int index = i * 3 + j;
            if (isnan(setPos[index])) {
                setPos[index] = vLastSetPos[index]; // Use last position if NaN
                cout << "-------------motor_angle_" << index << " NAN-----------" << endl;
                exit(0);
            } else {
                float delta = setPos[index] - vLastSetPos[index];
                if (delta > MORTOR_ANGLE_AMP) {
                    vLastSetPos[index] += MORTOR_ANGLE_AMP;
                    cout << "-------------motor_angle_" << index << " +MAX-----------" << endl;
                } else if (delta < -MORTOR_ANGLE_AMP) {
                    vLastSetPos[index] -= MORTOR_ANGLE_AMP;
                    cout << "-------------motor_angle_" << index << " -MAX-----------" << endl;
                } else {
                    vLastSetPos[index] = setPos[index]; // Update to current position
                }
            }
        }
    }
    float offSet[4]={0,0,0.1,0};
    // Ensure to set the last 4 positions correctly
    for (int i = 12; i < 16; ++i) {
        vLastSetPos[i] = setPos[i]+offSet[i-12];
    }

    motors.setPosition(vLastSetPos);
 
}

MatrixXf pinv(Eigen::MatrixXf  A,float pinvtoler)
{
    // std::cout << A << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXf X = Eigen::MatrixXf::Zero(col, row);
    Eigen::MatrixXf singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXf singularValues_inv_mat = Eigen::MatrixXf::Zero(col, row);
    for (long i = 0; i < k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);

        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

    //std::cout << X << std::endl;

    return X;

}

void writeMatrixToCSV(const std::string& filename, const Matrix<float, 4, 3>& matrix) {
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) file << ",";
        }
        file << "\n";
    }

    file.close();
}