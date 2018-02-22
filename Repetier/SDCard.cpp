/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Repetier.h"

#if SDSUPPORT

char tempLongFilename[LONG_FILENAME_LENGTH+1];
char fullName[LONG_FILENAME_LENGTH*SD_MAX_FOLDER_DEPTH+SD_MAX_FOLDER_DEPTH+1];

SDCard sd;

SDCard::SDCard()
{
    sdmode = false;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);

    //power to SD reader
#if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
#endif // SDPOWER > -1

#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    SET_INPUT(SDCARDDETECT);
    WRITE(SDCARDDETECT,HIGH);
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

} /// SDCard


void SDCard::automount()
{
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED)
    {
        if(sdactive)   // Card removed
        {
            if( Printer::debugInfo() )
            {
                Com::printFLN(PSTR("SD card removed"));
            }
#if UI_DISPLAY_TYPE!=0
            uid.executeAction(UI_ACTION_TOP_MENU);
#endif // UI_DISPLAY_TYPE!=0

            unmount();
            UI_STATUS( UI_TEXT_SD_REMOVED );
        }
    }
    else
    {
        if(!sdactive)
        {
            UI_STATUS( UI_TEXT_SD_INSERTED );
            if( Printer::debugInfo() )
            {
                Com::printFLN(PSTR("SD card inserted"));
            }
            Printer::setMenuMode(MENU_MODE_SD_MOUNTED,true);
            initsd();
#if UI_DISPLAY_TYPE!=0
            if(sdactive) {
                Printer::setAutomount(true);
                uid.executeAction(UI_ACTION_SD_PRINT+UI_ACTION_TOPMENU);
            }
#endif // UI_DISPLAY_TYPE!=0
        }
    }
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

} // automount


void SDCard::initsd()
{
    sdactive = false;

#if SDSS >- 1
#if defined(SDCARDDETECT) && SDCARDDETECT>-1
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED)
        return;
#endif // defined(SDCARDDETECT) && SDCARDDETECT>-1

    //fix in https://github.com/repetier/Repetier-Firmware/commit/d4e396d0f4d1b81cc4d388360be461f11ceb9edd ??
    fat.begin(SDSS, SPI_FULL_SPEED);  // dummy init of SD_CARD
    HAL::delayMilliseconds(50);       // wait for init end

    if(!fat.begin(SDSS,SPI_FULL_SPEED))
    {
        if( Printer::debugErrors() )
        {
            Com::printFLN(Com::tSDInitFail);
        }
        return;
    }
    sdactive = true;

    /* Das kann gar nicht mehr stimmen...
    if( uid.menuPos[uid.menuLevel] == 9 && uid.menuLevel == 2 )
    {
        // we are within the SD card menu at the moment - after the successful mounting, the menu shall point to the "print"/"mill" item and not to the "delete" item
        uid.menuPos[uid.menuLevel] = 0;
    }
    */

    Printer::setMenuMode(MENU_MODE_SD_MOUNTED,true);

    fat.chdir();
    if(selectFile((char *)"init.g",true))
    {
        startPrint();
    }
#endif // SDSS >- 1

} // initsd


void SDCard::mount()
{
    sdmode = false;
    initsd();

} // mount


void SDCard::unmount()
{
    sdmode = false;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);
    Printer::setMenuMode(MENU_MODE_SD_MOUNTED+MENU_MODE_PAUSED+MENU_MODE_SD_PRINTING,false);

#if UI_DISPLAY_TYPE!=0
    uid.cwd[0]='/';
    uid.cwd[1]=0;
    uid.folderLevel=0;
#endif // UI_DISPLAY_TYPE!=0

} // unmount


void SDCard::startPrint()
{
    if(!sdactive) return;
    sdmode = true;
    Printer::setPrinting(true);
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
} // startPrint

void SDCard::writeCommand(GCode *code)
{
    unsigned int    sum1=0, sum2=0; // for fletcher-16 checksum
    uint8_t         buf[100];
    uint8_t         p=2;
    int             params = 128 | (code->params & ~1);

    /*
    https://github.com/repetier/Repetier-Firmware/blob/master/src/ArduinoDUE/Repetier/SDCard.cpp 
    has fixes for 
    warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
    */  

    file.writeError = false;
    memcopy2(buf,&params);
    //*(int*)buf = params;
    
    if(code->isV2())   // Read G,M as 16 bit value
    {        
        memcopy2(&buf[p],&code->params2);
        //*(int*)&buf[p] = code->params2;
        
        p+=2;
        if(code->hasString())
            buf[p++] = strlen(code->text);
        if(code->hasM())
        {
            memcopy2(&buf[p],&code->M);
            //*(int*)&buf[p] = code->M;
            p+=2;
        }
        if(code->hasG())
        {
            memcopy2(&buf[p],&code->G);
            //*(int*)&buf[p]= code->G;
            p+=2;
        }
    }
    else
    {
        if(code->hasM())
        {
            buf[p++] = (uint8_t)code->M;
        }
        if(code->hasG())
        {
            buf[p++] = (uint8_t)code->G;
        }
    }
    if(code->hasX())
    {
        memcopy4(&buf[p],&code->X);
        //*(float*)&buf[p] = code->X;
        p+=4;
    }
    if(code->hasY())
    {
        memcopy4(&buf[p],&code->Y);
        //*(float*)&buf[p] = code->Y;
        p+=4;
    }
    if(code->hasZ())
    {
        memcopy4(&buf[p],&code->Z);
        //*(float*)&buf[p] = code->Z;
        p+=4;
    }
    if(code->hasE())
    {
        memcopy4(&buf[p],&code->E);
        //*(float*)&buf[p] = code->E;
        p+=4;
    }
    if(code->hasF())
    {
        memcopy4(&buf[p],&code->F);
        //*(float*)&buf[p] = code->F;
        p+=4;
    }
    if(code->hasT())
    {
        buf[p++] = code->T;
    }
    if(code->hasS())
    {
        memcopy4(&buf[p],&code->S);
        //*(long int*)&buf[p] = code->S;
        //*(int32_t*)&buf[p] = code->S;
        p+=4;
    }
    if(code->hasP())
    {       
        memcopy4(&buf[p],&code->P);
        //*(int32_t*)&buf[p] = code->P;
        //*(long int*)&buf[p] = code->P;
        p+=4;
    }
    if(code->hasI())
    {
        memcopy4(&buf[p],&code->I);
        //*(float*)&buf[p] = code->I;
        //*(float*)&buf[p] = code->I;
        p+=4;
    }
    if(code->hasJ())
    {       
        memcopy4(&buf[p],&code->J);
        //*(float*)&buf[p] = code->J;
        //*(float*)&buf[p] = code->J;
        p+=4;
    }
    if(code->hasString())   // read 16 uint8_t into string
    {
        char *sp = code->text;
        if(code->isV2())
        {
            uint8_t i = strlen(code->text);
            for(; i; i--) buf[p++] = *sp++;
        }
        else
        {
            for(uint8_t i=0; i<16; ++i) buf[p++] = *sp++;
        }
    }
    uint8_t *ptr = buf;
    uint8_t len = p;
    while (len)
    {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do
        {
            sum1 += *ptr++;
            if(sum1>=255) sum1-=255;
            sum2 += sum1;
            if(sum2>=255) sum2-=255;
        }
        while (--tlen);
    }
    buf[p++] = sum1;
    buf[p++] = sum2;
    if(params == 128)
    {
/*      if( Printer::debugErrors() )
        {
            Com::printErrorFLN(Com::tAPIDFinished);
        }
*/    }
    else
        file.write(buf,p);

    if (file.writeError)
    {
        if( Printer::debugErrors() )
        {
            Com::printFLN(Com::tErrorWritingToFile);
        }
    }
} // writeCommand


char *SDCard::createFilename(char *buffer,const dir_t &p)
{
    char *pos = buffer,*src = (char*)p.name;


    for (uint8_t i = 0; i < 11; i++,src++)
    {
        if (*src == ' ') continue;
        if (i == 8)
            *pos++ = '.';
        *pos++ = *src;
    }
    *pos = 0;
    return pos;

} // createFilename


bool SDCard::showFilename(const uint8_t *name)
{
    if (*name == DIR_NAME_DELETED || *name == '.') return false;
    return true;

} // showFilename


int8_t RFstricmp(const char* s1, const char* s2)
{
    while(*s1 && (tolower(*s1) == tolower(*s2)))
        s1++,s2++;
    return (const uint8_t)tolower(*s1)-(const uint8_t)tolower(*s2);

} // RFstricmp


int8_t RFstrnicmp(const char* s1, const char* s2, size_t n)
{
    while(n--)
    {
        if(tolower(*s1)!=tolower(*s2))
            return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
        s1++;
        s2++;
    }
    return 0;

} // RFstrnicmp


void SDCard::ls()
{
    SdBaseFile file;


    if( Printer::debugInfo() )
    {
        Com::printFLN(Com::tBeginFileList);
    }
    fat.chdir();

    file.openRoot(fat.vol());
    file.ls(0, 0);

    if( Printer::debugInfo() )
    {
        Com::printFLN(Com::tEndFileList);
    }

} // ls


bool SDCard::selectFile(char *filename, bool silent)
{
    SdBaseFile  parent;
    char*       oldP = filename;
    //boolean       bFound;


    if(!sdactive) return false;
    sdmode = false;

    file.close();

    parent = *fat.vwd();
    if (file.open(&parent, filename, O_READ))
    {
        if ((oldP = strrchr(filename, '/')) != NULL)
            oldP++;
        else
            oldP = filename;

        if(!silent)
        {
            if( Printer::debugInfo() )
            {
                Com::printF(Com::tFileOpened, oldP);
                Com::printFLN(Com::tSpaceSizeColon,file.fileSize());
            }
        }
        sdpos = 0;
        filesize = file.fileSize();

        if( Printer::debugInfo() )
        {
            Com::printFLN(Com::tFileSelected);
        }
        return true;
    }
    else
    {
        if(!silent)
        {
            if( Printer::debugInfo() )
            {
                Com::printFLN(Com::tFileOpenFailed);
            }
        }
        return false;
    }

} // selectFile


void SDCard::printStatus()
{
    if( Printer::debugInfo() )
    {
        if(sdactive)
        {
            Com::printF(Com::tSDPrintingByte,sdpos);
            Com::printFLN(Com::tSlash,filesize);
        }
        else
        {
            Com::printFLN(Com::tNotSDPrinting);
        }
    }

} // printStatus


void SDCard::startWrite(char *filename)
{
    if(!sdactive) return;
    file.close();
    sdmode = false;
    fat.chdir();

    if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
    {
        if( Printer::debugErrors() )
        {
            Com::printFLN(Com::tOpenFailedFile,filename);
        }
    }
    else
    {
        UI_STATUS( UI_TEXT_UPLOADING );
        savetosd = true;

        if( Printer::debugInfo() )
        {
            Com::printFLN(Com::tWritingToFile,filename);
        }
    }

} // startWrite


void SDCard::finishWrite()
{
    if(!savetosd) return; // already closed or never opened
    file.sync();
    file.close();
    savetosd = false;

    if( Printer::debugInfo() )
    {
        Com::printFLN(Com::tDoneSavingFile);
    }

    g_uStartOfIdle = HAL::timeInMilliseconds();

} // finishWrite


void SDCard::deleteFile(char *filename)
{
    if(!sdactive) return;

    if(Printer::isMenuMode(MENU_MODE_SD_PRINTING))
    {
        // we do not allow to delete a file while we are printing/milling from the SD card
        if( Printer::debugErrors() )
        {
            Com::printFLN(PSTR("It is not possible to delete a file from the SD card until the current processing has finished."));
        }

        showError( (void*)ui_text_delete_file, (void*)ui_text_operation_denied );
        return;
    }

    sdmode = false;
    file.close();
    if(fat.remove(filename))
    {
        if( Printer::debugInfo() )
        {
            Com::printFLN(Com::tFileDeleted);
        }
    }
    else
    {
        if(fat.rmdir(filename))
        {
            if( Printer::debugInfo() )
            {
                Com::printFLN(Com::tFileDeleted);
            }
        }
        else
        {
            if( Printer::debugErrors() )
            {
                Com::printFLN(Com::tDeletionFailed);
            }
        }
    }

} // deleteFile


void SDCard::makeDirectory(char *filename)
{
    if(!sdactive) return;
    sdmode = false;
    file.close();

    if(fat.mkdir(filename))
    {
        if( Printer::debugInfo() )
        {
            Com::printFLN(Com::tDirectoryCreated);
        }
    }
    else
    {
        if( Printer::debugErrors() )
        {
            Com::printFLN(Com::tCreationFailed);
        }
    }

} // makeDirectory


#ifdef GLENN_DEBUG
void SDCard::writeToFile()
{
    size_t  nbyte;
    char    szName[10];

    strcpy(szName, "Testing\r\n");
    nbyte = file.write(szName, strlen(szName));

    if( Printer::debugInfo() )
    {
        Com::print("L=");
        Com::print((long)nbyte);
        Com::println();
    }

} // writeToFile
#endif // GLENN_DEBUG

#endif // SDSUPPORT
