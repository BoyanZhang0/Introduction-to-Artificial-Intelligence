/////////////////////////////////////////////////////////////////////////////////////////////
//�ļ�����		CSearchPlan_RunForward.cpp
//��  �ܣ�		����ǰ�������Ĺ滮
//�����ߣ�		��־ǿ
//��  �ڣ�		2022��10��09��
//��  �£�		2022��10��12��
//��  �£�		2022��10��21��
//��  �ȣ�		40��
/////////////////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"									//MFC��׼ͷ�ļ�
#include "CSearchPlan.h"							//���������Ĺ滮��ͷ�ļ�


//��  �ƣ�		RunForward()
//��  �ܣ�		����ǰ�������Ĺ滮
//��  ����		��
//����ֵ��		��
void CSearchPlan::RunForward()
{
	//����OPEN�б�Ԫ��Ϊ��㡣���ù������
	vector<NODE> OPEN;
	//����CLOSEDӳ�䡣Ԫ��Ϊ״̬�����ַ���-���ID
	map <CString,unsigned int> CLOSED;			//�����ж��Ƿ��ظ����
	//���������Tr��Ԫ��Ϊ�ս��ID-��
	map<unsigned int, EDGE> Tr;


	////////////////////////////////////////////////////////////////////////
	//����1����ʼ��
	//��ʼ��OPEN����	
	//�����ʼ״̬/���
	NODE node;
	node.state = m_InitState;	//��ʼ״̬
	node.nDepth = 0;			//������Ϊ0
	node.nID = m_nStateID;		//���IDΪ0
	OPEN.push_back(node);		//��ӵ�OPEN�б�

	
	///////////////////////////////////////////////////////////////////////
	//����2����ʼ������ֱ��OPEN�б�Ϊ��
	//while (OPEN.size() != 0)
	//{
		/**
		1	OPEN�б��һ��Ԫ�س��ӣ������б���ɾ��
		2	�����ǰ״̬��Ŀ��״̬��ʹ��CSearchPlan::IsStateSmall:
		3		���ݵõ��滮��ʹ��CSearchPlan::BackTrack����return
		4	���򣬼�������:
		5		�����ǰ״̬��CLOSEDӳ���в�����:
		6			����ǰ�����ӵ�CLOSED
		7			������ID��0��������
		8			չ�����״̬��OPEN��ʹ��CSearchPlan::Expand
		**/

		// TODO
	//}
	while (OPEN.size() != 0) 
	{
        auto temp = OPEN.front();
        OPEN.erase(OPEN.begin());
        if (IsStateSmall(m_GoalState, temp.state))
		{
            BackTrack(temp.nID, Tr);
            return;
        }
        else
		{
            if (CLOSED.find(GetStateIndex(temp.state)) == CLOSED.end())
			{
                CLOSED.emplace(GetStateIndex(temp.state), m_nStateID);
                if (m_nStateID == 0) m_nStateID++;
                Expand(temp, OPEN, CLOSED, Tr);
            }
        }
    }
}