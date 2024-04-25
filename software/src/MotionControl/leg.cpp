#include "leg.h"
/****************legStatus**********************/
CLeg::CLeg(enum_LEGNAME name,float L1,float L2,float L3)
{
    m_sName=name;
    m_fL1=L1/1000.0; // mm->m
    m_fL2=L2/1000.0;
    m_fL3=L3/1000.0;
    m_svStatus=false;
}
/**
 * @brief update m_fTheta with jointPos (preseent or cmd)
 * 
 * @param jointPos 
 */
 void CLeg::SetJointPos(Matrix<float,3,1> jointPos)
 {
    m_fTheta1=jointPos[0];
    m_fTheta2=jointPos[1];
    m_fTheta3=jointPos[2];
 }

 void CLeg::ChangeStatus(enum_LEGSTATUS legStatus)
 {
    m_eLegStatus=legStatus;
 }

enum_LEGSTATUS CLeg::GetLegStatus()
{
    return m_eLegStatus;
}

Matrix<float, 3, 3> CLeg::GetJacobian()
{
    return m_mfJacobian;
}

/**************Kinematics*******************/
/**
 * @brief 
 * Calculcate jacobian_vector with jointPresPos
 */
void CLeg::UpdateJacobian()
{
        // cout<<"theta : "<<m_fTheta1<<","<<m_fTheta2<<","<<m_fTheta3<<endl;   
       Matrix<float, 3, 3>  CHANGE; CHANGE << 0, 0, 1,
        1, 0, 0,
        0, 1, 0;
        float factor_y, factor_z, factor_By, factor_Cy, factor_Bz, factor_Cz, factor_Ax, factor_Bx;
        switch (m_sName)
        {
        case LF:
            factor_By = -1;
            factor_Cy = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case RF:
            factor_By = 1;
            factor_Cy = -1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        case LH:
            factor_By = 1;
            factor_Cy = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case RH:
            factor_By = -1;
            factor_Cy = -1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
        m_mfJacobian(0, 0) = factor_y * (-sin(m_fTheta1) * cos(m_fTheta2) * m_fL1 + factor_By * sin(m_fTheta1) * sin(m_fTheta2 + m_fTheta3) * m_fL2 + factor_Cy* cos(m_fTheta1) * m_fL3);
        m_mfJacobian(0, 1) = factor_y * (-cos(m_fTheta1) * sin(m_fTheta2) * m_fL1 - factor_By * cos(m_fTheta1) * cos(m_fTheta2 + m_fTheta3) * m_fL2);
        m_mfJacobian(0, 2) = factor_y * (-factor_By * cos(m_fTheta1) * cos(m_fTheta2 + m_fTheta3) * m_fL2);
        m_mfJacobian(1, 0) = factor_z * (cos(m_fTheta1) * cos(m_fTheta2) * m_fL1 + factor_Bz * cos(m_fTheta1) * sin(m_fTheta2 + m_fTheta3) * m_fL2 + factor_Cz * sin(m_fTheta1) * m_fL3);
        m_mfJacobian(1, 1) = factor_z * (-sin(m_fTheta1) * sin(m_fTheta2) * m_fL1 + factor_Bz * sin(m_fTheta1) * cos(m_fTheta2 + m_fTheta3) * m_fL2);
        m_mfJacobian(1, 2) = factor_z * (factor_Bz * sin(m_fTheta1) * cos(m_fTheta2 + m_fTheta3) * m_fL2);
        m_mfJacobian(2, 0) = 0;
        m_mfJacobian(2, 1) = factor_Ax * cos(m_fTheta2) * m_fL1 + factor_Bx * sin(m_fTheta2 + m_fTheta3) * m_fL2;
        m_mfJacobian(2, 2) = factor_Bx * sin(m_fTheta2 + m_fTheta3) * m_fL2;
        m_mfJacobian = CHANGE * m_mfJacobian;
       // m_mfJacobian = m_mfJacobian/1000; // mm -> m
        //  if(m_sName==LF)
        //  cout<<"lf1: "<<m_mfJacobian<<endl;
        // cout<<"lf1: "<<m_fTheta1<<"/r lf2: "<<m_fTheta2<<"/r lf3: "<<m_fTheta3<<endl;
        
    
}

Matrix<float,3,1> CLeg::ForwardKinematic()
{
        Matrix<float,3,1> legPos;
        float factor_Ay, factor_By, factor_Cy, factor_Az, factor_Bz, factor_Cz, factor_Ax, factor_Bx;//The sign factors before the formulas m_fL1, m_fL2, and m_fL3, where A represents the sign factors before m_fL1, B represents the sign factors before m_fL2, C represents the sign factors before m_fL3
        float factor_y, factor_z;//the sign factors of whole formulas of y,z
        switch (m_sName)
        {
        case LF:
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case RF:
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        case LH:
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case RH:
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
        // represent y,z,x
        legPos(1,0) = factor_y * (factor_Ay * cos(m_fTheta1) * cos(m_fTheta2) * m_fL1 + factor_By * cos(m_fTheta1) * sin(m_fTheta2 + m_fTheta3) * m_fL2 + factor_Cy*sin(m_fTheta1) * m_fL3);
        legPos(2,0) = factor_z * (factor_Az * sin(m_fTheta1) * cos(m_fTheta2) * m_fL1 + factor_Bz * sin(m_fTheta1) * sin(m_fTheta2 + m_fTheta3) * m_fL2 + factor_Cz * cos(m_fTheta1) * m_fL3);
        legPos(0,0) = factor_Ax * sin(m_fTheta2) * m_fL1 + factor_Bx * cos(m_fTheta2 + m_fTheta3) * m_fL2;
        return legPos;
}

void CLeg::setTouchStatus(bool status)
{
    m_touchStatus=status;
}
bool  CLeg::getTouchStatus()
{
    return m_touchStatus;
}
/**
 * @brief inverse Kinematics
 * 
 * @param cmdpos 
 * Calculcate joint angles (jointCmdPos) for motors with foot position(cmdpos) in shoulder coordinate
 */
Matrix<float,3,1> CLeg::InverseKinematic(Matrix<float, 1, 3> cmdpos)
{
        Matrix<float,3,1> jointCmdPos;
        float factor_x, factor_y, factor_z, factor_1, factor_2, factor_0;
        float x, y, z;
        x = cmdpos(0, 0);
        y = cmdpos(0, 1);
        z = cmdpos(0, 2);
        switch (m_sName)
        {
        case LF:
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = -1;
            factor_0 = 1;
            break;
        case RF:
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case LH:
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case RH:
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = -1;
            factor_0 = 1;
            break;

        }
        // jointCmdPos(legNum, 1) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(m_fL3, sqrt(z * z + y * y - m_fL3 * m_fL3));
        // jointCmdPos(legNum, 2) = factor_2 * asin((m_fL1 * m_fL1 + m_fL2 * m_fL2 + m_fL3 * m_fL3 - x * x - y * y - z * z) / (2 * m_fL1 * m_fL2));
        // jointCmdPos(legNum, 0) = atan2(factor_x * x, sqrt((m_fL1 + factor_0 * sin(jointCmdPos(legNum, 2)) * m_fL2) * (m_fL1 + factor_0 * sin(jointCmdPos(legNum, 2)) * m_fL2) + (cos(jointCmdPos(legNum, 2)) * m_fL2) * (cos(jointCmdPos(legNum, 2)) * m_fL2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(legNum, 2)) * m_fL2, m_fL1 + factor_0 * m_fL2 * sin(jointCmdPos(legNum, 2)));
         jointCmdPos(0, 0) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(m_fL3, sqrt(z * z + y * y - m_fL3 * m_fL3));
         jointCmdPos(2, 0) = factor_2 * asin((m_fL1 * m_fL1 + m_fL2 * m_fL2 + m_fL3 * m_fL3 - x * x - y * y - z * z) / (2 * m_fL1 * m_fL2));
         jointCmdPos(1, 0) = atan2(factor_x * x, sqrt((m_fL1 + factor_0 * sin(jointCmdPos(2, 0)) * m_fL2) * (m_fL1 + factor_0 * sin(jointCmdPos(2, 0)) * m_fL2) + (cos(jointCmdPos(2, 0)) * m_fL2) * (cos(jointCmdPos(2, 0)) * m_fL2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(2, 0)) * m_fL2, m_fL1 + factor_0 * m_fL2 * sin(jointCmdPos(2, 0)));

         return jointCmdPos;
    
}

