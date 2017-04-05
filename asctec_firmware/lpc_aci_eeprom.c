#include "lpc_aci_eeprom.h"

int anz_param_saved = 0;

struct ee_data content[COUNT_PAGES];

void lpc_aci_init(void)
{
	int j=0;
	int i=0;
	for( j=0;j<COUNT_PAGES; j++)
		for(i=0; i<254 ; i++) content[j].data[i]=0xFF;
	aciSetSaveParaCallback(lpc_aci_SavePara);
	aciSetReadParafromFlashCallback(lpc_aci_ReadParafromFlash);
	aciSetWriteParatoFlashCallback(lpc_aci_WriteParatoFlash);
}

short lpc_aci_ReadParafromFlash(void)
{
	unsigned int command_ee,response_ee[2];
	int cnt=0;
	short temp_id;
	unsigned char temp_vartype;
	struct ee_data *ee_data_ptr;
	int para_load = 0;
	int k = 0;
	command_ee=0;
	ee_readn(command_ee,response_ee);

	short counting = 0;

	ee_data_ptr= (struct ee_data *) response_ee[1];
	memcpy(&content[0],ee_data_ptr,sizeof(struct ee_data));
	para_load++;
	while((!content[0].next_side) && (para_load<COUNT_PAGES)  )
	{

		ee_readn(para_load,response_ee);
		if(response_ee[0]!=0) break;
		ee_data_ptr= (struct ee_data *) response_ee[1];
		memcpy(&(content[para_load]),ee_data_ptr,sizeof(struct ee_data));
		para_load++;
	}
	if((content[0].next_side!=1) && (content[0].next_side!=0) ) return (short) content[0].next_side ;
	para_load=0;
	unsigned char next_side_byte = 0;
	while((!next_side_byte)){
		cnt=0;

		while(cnt<content[para_load].data_count)
		{
			memcpy(&temp_id,&content[para_load].data[cnt],2);
			cnt+=2;
			memcpy(&temp_vartype,&content[para_load].data[cnt],1);
			cnt+=1;

			for(k=0;k<aciListParCount;k++){
				if(aciListPar[k].id==temp_id)
				{
					if(aciListPar[k].varType==temp_vartype)
					{
						memcpy(aciListPar[k].ptrToVar,&content[para_load].data[cnt],temp_vartype >> 2);
						counting++;
					}
					break;
				}
			}
			cnt+=temp_vartype >> 2;
		}
		next_side_byte=content[para_load].next_side;
		para_load++;
	}

	return counting;
}

void lpc_aci_SavePara(void)
{
	int cnt = 0;
	anz_param_saved=0;
	int para_load=0;
	int k=0;

	for(k=0;k<aciListParCount;k++) {
		if((cnt+4+(aciListPar[k].varType >> 2))>253)
		{
			content[para_load].data_count=cnt;
			content[para_load].next_side=0;
			para_load++;
			if(para_load==COUNT_PAGES) break;
			cnt=0;
		}
		memcpy(&content[para_load].data[cnt], &aciListPar[k].id, 2);
		cnt += 2;
		memcpy(&content[para_load].data[cnt], &aciListPar[k].varType, 1);
		cnt += 1;
		memcpy(&content[para_load].data[cnt], aciListPar[k].ptrToVar, aciListPar[k].varType >> 2);
		cnt += 	aciListPar[k].varType >> 2;
		anz_param_saved++;
	}

	content[para_load].data_count=cnt;
	content[para_load].next_side=1;
}

short lpc_aci_WriteParatoFlash(void)
{
	unsigned int command_ee,response_ee[2];
	//erase eeprom
	ee_erase(command_ee,response_ee);

	int para_load = 0;
	unsigned char next_side_byte = 0;

	while((!next_side_byte)){
		command_ee=(unsigned int) (&content[0]);
		ee_write(command_ee,response_ee);
		if(response_ee[0]==501) return 501;
		next_side_byte = content[para_load].next_side;
		para_load++;
	}

	return anz_param_saved;
}
