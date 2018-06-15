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
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];

SDCard sd;

SDCard::SDCard() {
    sdmode = 0;
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
    if(READ(SDCARDDETECT) != SDCARDDETECTINVERTED){
        if(sdactive || sdmode == 100)   // Card removed
        {
            Com::printFLN(PSTR("SD card removed"));
#if UI_DISPLAY_TYPE!=0
            uid.exitmenu();
#endif // UI_DISPLAY_TYPE!=0
            unmount();
            UI_STATUS( UI_TEXT_SD_REMOVED );
        }
    } else {
        if(!sdactive && sdmode != 100) {
            UI_STATUS( UI_TEXT_SD_INSERTED );
            mount();
            if(sdmode != 100) // send message only if we have success
                Com::printFLN(PSTR("SD card inserted")); // Not translatable or host will not understand signal
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
    HAL::delayMilliseconds(50);       // wait for stabilization of contacts, bootup ...
    fat.begin(SDSS, SD_SCK_MHZ(50));  // dummy init of SD_CARD
    HAL::delayMilliseconds(50);       // wait for init end

    if(!fat.begin(SDSS, SD_SCK_MHZ(50))) {
        Com::printFLN(Com::tSDInitFail);
        sdmode = 100; // prevent automount loop!
        if (fat.card()->errorCode()) {
            Com::printFLN(PSTR("\nSD initialization failed.\n"
                               "Do not reformat the card!\n"
                               "Is the card correctly inserted?\n"
                               "Is chipSelect set to the correct value?\n"
                               "Does another SPI device need to be disabled?\n"
                               "Is there a wiring/soldering problem?"));
            Com::printFLN(PSTR("errorCode: "), int(fat.card()->errorCode()));
            
            char szStatus[21];
            strcpy( szStatus, UI_TEXT_SD_ERROR );
            addLong( szStatus, int(fat.card()->errorCode()), 3 );
            switch(fat.card()->errorCode()){
                case 32:
                {
                    strcat( szStatus, " Timeout" );
                    Com::printFLN(PSTR("SD Timeout"));
                    break;
                }
                default:
                {
                    break;
                }
            }
            UI_STATUS_UPD_RAM( szStatus );
            return;
        }
        if (fat.vol()->fatType() == 0) {
            Com::printFLN(PSTR("Can't find a valid FAT16/FAT32 partition.\n"));
            char szStatus[21];
            strcpy( szStatus, UI_TEXT_SD_ERROR );
            strcat( szStatus, "FAT16/32 not" );
            UI_STATUS_UPD_RAM( szStatus );
            return;
        }
        if (!fat.vwd()->isOpen()) {
            Com::printFLN(PSTR("Can't open root directory.\n"));
            char szStatus[21];
            strcpy( szStatus, UI_TEXT_SD_ERROR );
            strcat( szStatus, " root dir" );
            UI_STATUS_UPD_RAM( szStatus );
            return;
        }
        return;
    }
    Com::printFLN(PSTR("Card successfully initialized."));
    sdactive = true;
    Printer::setMenuMode(MENU_MODE_SD_MOUNTED,true);

    fat.chdir();
    if(selectFileByName("init.g", true))
    {
        startPrint();
    }
#endif // SDSS >- 1
} // initsd

void SDCard::mount()
{
    sdmode = 0;
    initsd();
} // mount

void SDCard::unmount()
{
    sdmode = 0;
    sdactive = false;
    savetosd = false;
    Printer::setAutomount(false);
    Printer::setMenuMode(MENU_MODE_SD_MOUNTED + MENU_MODE_PAUSED + MENU_MODE_SD_PRINTING, false);
#if UI_DISPLAY_TYPE!=0
    uid.cwd[0] = '/';
    uid.cwd[1] = 0;
    uid.folderLevel = 0;
#endif // UI_DISPLAY_TYPE!=0
} // unmount

void SDCard::startPrint()
{
    if(!sdactive) return;
    sdmode = 1;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, true);
    Printer::setMenuMode(MENU_MODE_PAUSED, false);
    Printer::setPrinting(true);
} // startPrint

void SDCard::writeCommand(GCode *code)
{
    unsigned int    sum1=0, sum2=0; // for fletcher-16 checksum
    uint8_t         buf[100];
    uint8_t         p=2;
    file.clearWriteError();
    uint16_t        params = 128 | (code->params & ~1);
    memcopy2(buf, &params);
    
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
        Com::printErrorFLN(Com::tAPIDFinished);
    }
    else
        file.write(buf,p);

    if (file.getWriteError())
    {
        Com::printFLN(Com::tErrorWritingToFile);
    }
} // writeCommand

char *SDCard::createFilename(char *buffer, const dir_t &p) {
    char *pos = buffer, *src = (char*)p.name;

    for (uint8_t i = 0; i < 11; i++,src++) {
        if (*src == ' ') continue;
        if (i == 8)
            *pos++ = '.';
        *pos++ = *src;
    }
    *pos = 0;
    
    return pos;
} // createFilename

bool SDCard::showFilename(const uint8_t *name) {
    if (*name == DIR_NAME_DELETED || *name == '.') return false;
    return true;
}

void SDCard::ls()
{
    SdBaseFile file;

    Com::printFLN(Com::tBeginFileList);
    fat.chdir();

    file.openRoot(fat.vol());
    file.ls(0, 0);
    Com::printFLN(Com::tEndFileList);
} // ls

bool SDCard::selectFileByName(const char* filename, bool silent) {
    if (!sdactive)
        return false;
    sdmode = 0;

    const char* oldP = filename;
    file.close();
    // Filename for progress view
    // strncpy(Printer::printName, filename, 20);
    // Printer::printName[20] = 0;
    if (file.open(fat.vwd(), filename, O_READ)) {
        if ((oldP = strrchr(filename, '/')) != NULL)
            oldP++;
        else
            oldP = filename;

        if (!silent) {
            Com::printF(Com::tFileOpened, oldP);
            Com::printFLN(Com::tSpaceSizeColon, file.fileSize());
        }
        sdpos = 0;
        filesize = file.fileSize();
        Com::printFLN(Com::tFileSelected);
        return true;
    } else {
        if (!silent)
            Com::printFLN(Com::tFileOpenFailed);
        return false;
    }
}

bool SDCard::selectFileByPos(uint16_t filePos, bool silent) {
    if (!sdactive)
        return false;
    sdmode = 0;

    FatFile *root = sd.fat.vwd();
    FatFile ffile;
    root->rewind();
    while (ffile.openNext(root, O_READ))
    {
        ffile.getName(tempLongFilename, LONG_FILENAME_LENGTH);
        if (uid.folderLevel >= SD_MAX_FOLDER_DEPTH && strcmp(tempLongFilename, "..") == 0) {
            ffile.close();
            continue;
        }
        if (tempLongFilename[0] == '.' && tempLongFilename[1] != '.') {
            ffile.close();
            continue; // MAC CRAP
        }
        if (filePos-- > 0) { //loop until filePos reached
            ffile.close();
            continue;
        }

        if (file.open(fat.vwd(), ffile.dirIndex(), O_READ)) {
            sdpos = 0;
            filesize = file.fileSize();
            if (!silent) {
                Com::printF(Com::tFileSelected, (int32_t)ffile.dirIndex());
                Com::printFLN(Com::tSpaceSizeColon, file.fileSize());
            }

            ffile.close();
            return true;
        } else {
            if (!silent)
                Com::printFLN(Com::tFileOpenFailed);
        }
    }

    ffile.close();
    return false;
} // getSDFilenameAt


void SDCard::printStatus() {
    if(sdactive) {
        Com::printF(Com::tSDPrintingByte, sdpos);
        Com::printFLN(Com::tSlash, filesize);
    } else {
        Com::printFLN(Com::tNotSDPrinting);
    }
}

void SDCard::startWrite(char *filename) {
    if(!sdactive) return;
    file.close();
    sdmode = 0;
    fat.chdir();
    if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
        Com::printFLN(Com::tOpenFailedFile, filename);
    } else {
        UI_STATUS( UI_TEXT_UPLOADING );
        savetosd = true;
        Com::printFLN(Com::tWritingToFile, filename);
    }
}

void SDCard::finishWrite() {
    if(!savetosd) return; // already closed or never opened
    file.sync();
    file.close();
    savetosd = false;

    Com::printFLN(Com::tDoneSavingFile);

    g_uStartOfIdle = HAL::timeInMilliseconds(); //SDCard::finishWrite() tDoneSavingFile
} // finishWrite

void SDCard::deleteFile(char *filename) {
    if(!sdactive) return;
    sdmode = 0;
    file.close();
    if(fat.remove(filename)) {
        Com::printFLN(Com::tFileDeleted);
    } else {
        if(fat.rmdir(filename))
            Com::printFLN(Com::tFileDeleted);
        else
            Com::printFLN(Com::tDeletionFailed);
    }
}

void SDCard::makeDirectory(char *filename) {
    if(!sdactive) return;
    sdmode = 0;
    file.close();
    if(fat.mkdir(filename)) {
        Com::printFLN(Com::tDirectoryCreated);
    } else {
        Com::printFLN(Com::tCreationFailed);
    }
}

#endif // SDSUPPORT
