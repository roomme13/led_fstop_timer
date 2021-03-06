/*
    Copyright (C) 2011-2012 William Brodie-Tyrrell
    william@brodie-tyrrell.org
  
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of   
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "Program.h"

void Program::clear()
{
    // base
    steps[0].stops=300;
    steps[0].grade=100;
    strcpy(steps[0].text, "Base Exposure");
    isstrip=false;
   
    // invalid
    for(int i=1;i<MAXSTEPS;++i){
        steps[i].stops=0;
        steps[i].grade=100;
        strcpy(steps[i].text, "Undefined");
    }

	clearExposures();
}

void Program::clearExposures()
{
    // invalid
    for(int i=0;i<MAXEXPOSURES;++i){
        exposures[i].ms=0;
    }
}

void Program::configureStrip(int base, int step, bool cov, unsigned char grade, Paper& p)
{
    isstrip=true;
    cover=cov;

    int expos=base;
    for(char i=0;i<MAXSTEPS;++i) {
        steps[i].stops=expos;
        steps[i].grade=grade;
        strcpy(steps[i].text, "Strip ");
        dtostrf(0.01f*expos, 1, 2, &steps[i].text[6]);
        strcpy(&steps[i].text[10], cov ? " Cov" : " Ind");
        expos+=step;
    }
}

void Program::clipExposures()
{
    // don't want to be here forever or overflow the screen
    for(char i=0;i<MAXEXPOSURES;++i){
        if(exposures[i].ms > MAXMS) 
            exposures[i].ms = MAXMS;
    }
}

bool Program::compile(char dryval, bool splitgrade, Paper& p)
{
	clearExposures();
	
    if(isstrip){
        if(cover)
            compileStripCover(dryval, p);
        else
            compileStripIndiv(dryval, p);

        clipExposures();
        return true;
    }
    else{
        return compileNormal(dryval, splitgrade, p);
    }
}

void Program::compileStripIndiv(char dryval, Paper& p)
{
    for(int i=0;i<MAXSTEPS;++i){
        exposures[i].ms=hunToMillis(steps[i].stops-dryval);
	    exposures[i].softpower=p.getAmountSoft(steps[i].grade);
	    exposures[i].hardpower=p.getAmountHard(steps[i].grade);
    	exposures[i].step = &steps[i];
    }
}

void Program::compileStripCover(char dryval, Paper& p)
{
    unsigned long sofar=0;

    for(int i=0;i<MAXSTEPS;++i){
        unsigned long thisexp=hunToMillis(steps[i].stops-dryval);
        exposures[i].ms=thisexp-sofar;
	    exposures[i].softpower=p.getAmountSoft(steps[i].grade);
	    exposures[i].hardpower=p.getAmountHard(steps[i].grade);
    	exposures[i].step = &steps[i];
        sofar+=exposures[i].ms;
    }
}

bool Program::compileNormal(char dryval, bool splitgrade, Paper& p)
{
    // Serial.println("compileNormal");

    // base exposure, perhaps with drydown
    int base=steps[0].stops-dryval;

    // Serial.print("steps[0].text:");
    // Serial.println(steps[0].text);
      
    exposures[0].ms=hunToMillis(base);
    exposures[0].softpower=p.getAmountSoft(steps[0].grade);
    exposures[0].hardpower=p.getAmountHard(steps[0].grade);
	exposures[0].step = &steps[0];

	if (splitgrade){
		exposures[0].step = &steps[0];
		exposures[1].step = &steps[0];

	    exposures[0].ms=hunToMillis(base);
	    exposures[1].ms=hunToMillis(base);

	    exposures[0].softpower=p.getAmountSoft(steps[0].grade);
	    exposures[0].hardpower=LEDDriver::LED_OFF;
	    exposures[1].softpower=LEDDriver::LED_OFF;
	    exposures[1].hardpower=p.getAmountHard(steps[0].grade);
	}else{
		exposures[0].step = &steps[0];

	    exposures[0].ms=hunToMillis(base);

	    exposures[0].softpower=p.getAmountSoft(steps[0].grade);
	    exposures[0].hardpower=p.getAmountHard(steps[0].grade);
	}
    
    // figure out the total exposure desired for each step (base+adjustment)
    unsigned long dodgetime=0;
    for(int i=1;i<MAXSTEPS;++i){
		int pos_multiplier = splitgrade ? 2 : 1;
		int j = i * pos_multiplier;
		
        if(steps[i].stops == 0){
            exposures[j].ms=0;
		    exposures[j].hardpower=LEDDriver::LED_OFF;
		    exposures[j].softpower=LEDDriver::LED_OFF;
			if (splitgrade){
	            exposures[j+1].ms=0;
			    exposures[j+1].hardpower=LEDDriver::LED_OFF;
			    exposures[j+1].softpower=LEDDriver::LED_OFF;
			}
            continue;
        }

		if (splitgrade){
			exposures[j].step = &steps[i];
			exposures[j+1].step = &steps[i];

		    exposures[j].ms=hunToMillis(base+steps[i].stops);
		    exposures[j+1].ms=hunToMillis(base+steps[i].stops);

		    exposures[j].softpower=p.getAmountSoft(steps[i].grade);
		    exposures[j].hardpower=LEDDriver::LED_OFF;
		    exposures[j+1].softpower=LEDDriver::LED_OFF;
		    exposures[j+1].hardpower=p.getAmountHard(steps[i].grade);
		}else{
			exposures[j].step = &steps[i];

		    exposures[j].ms=hunToMillis(base+steps[i].stops);

		    exposures[j].softpower=p.getAmountSoft(steps[i].grade);
		    exposures[j].hardpower=p.getAmountHard(steps[i].grade);
		}

        // is dodge?
        if(steps[i].stops < 0){
            // keep track of total time spent dodging          
            dodgetime+=exposures[0].ms-exposures[j].ms;  
            // convert to time-difference
            exposures[j].ms=exposures[0].ms-exposures[j].ms;
			if (splitgrade){
	            exposures[j+1].ms=exposures[0].ms-exposures[j+1].ms;
			}
        }
        else{
            // convert to time-difference
            exposures[j].ms-=exposures[0].ms;          
			if (splitgrade){
	            exposures[j+1].ms-=exposures[0].ms;          
			}
        }
    }
    
    // fail if we have more dodge than base exposure
    if(dodgetime > exposures[0].ms)
        return false;
    
    // take off the dodgetime
    exposures[0].ms-=dodgetime;
	if (splitgrade){
	    exposures[1].ms-=dodgetime;
	}
    
    clipExposures();

    return true;
}

void Program::Step::display(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    // print text
    disp.clearDisplay();
    disp.setTextSize(1); // Draw 2X-scale text
    disp.setTextColor(SSD1306_WHITE);
    disp.setCursor(0,0);
    disp.print(text);
    displayGrade(disp, buf, lin);
    displayTime(disp, buf, lin);
    disp.display();
}

void Program::Step::displayTime(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    disp.setCursor(0,2);

    // print stops
    char used=0;
    if(stops >= 0){
        disp.print("+");
        ++used;
    }
    dtostrf(0.01f*stops, 0, 2, buf);
    used+=strlen(buf);
    disp.print(buf);

    // print compiled seconds
    if(lin){
        disp.print("=");
        dtostrf(0.001f*50000, 0, 3, buf);
        used+=strlen(buf)+2;
        disp.print(buf);
        disp.print("s");
     
        // fill out to 18 chars with spaces
        // keep the 19th for 'S' splitgrade-indicator
        // keep the 20th for 'D' drydown-indicator
        for(int i=0;i<18-used;++i)
            buf[i]=' ';
        buf[18-used]='\0';
        disp.print(buf);
    }  
}

void Program::Step::displayGrade(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    disp.setCursor(0,1);

    disp.print("Grade: ");

    // print grade
    char used=0;
    itoa(grade, buf, 10);
    used+=strlen(buf);
    disp.print(buf);
}

void Program::Exposure::display(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    // print text
    disp.clearDisplay();
    disp.setTextSize(1); // Draw 2X-scale text
    disp.setTextColor(SSD1306_WHITE);
    disp.setCursor(0,0);
    disp.print(step->text);
    displayGrade(disp, buf, lin);
    displayTime(disp, buf, lin);
    disp.display();
}

void Program::Exposure::displayTime(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    disp.setCursor(0,2);

    // print stops
    char used=0;
    if(step->stops >= 0){
        disp.print("+");
        ++used;
    }
    dtostrf(0.01f*step->stops, 0, 2, buf);
    used+=strlen(buf);
    disp.print(buf);

    // print compiled seconds
    if(lin){
        disp.print("=");
        dtostrf(0.001f*ms, 0, 3, buf);
        used+=strlen(buf)+2;
        disp.print(buf);
        disp.print("s");
     
        // fill out to 18 chars with spaces
        // keep the 19th for 'S' splitgrade-indicator
        // keep the 20th for 'D' drydown-indicator
        for(int i=0;i<18-used;++i)
            buf[i]=' ';
        buf[18-used]='\0';
        disp.print(buf);
    }  
}
void Program::Exposure::displayGrade(Adafruit_SSD1306 &disp, char *buf, bool lin)
{
    disp.setCursor(0,1);

    disp.print("Grade: ");

    // print grade
    char used=0;

    itoa(step->grade, buf, 10);
    used+=strlen(buf);
    disp.print(buf);

    //ToDo: Calculate these percentages based on the paper and the grade
    disp.print(" ");    
    disp.print(softpower);    
    disp.print(":");    
    disp.print(hardpower);    
}

Program::Step &Program::getStep(int which)
{
    return steps[which]; 
}

Program::Exposure &Program::getExposure(int which)
{
    return exposures[which]; 
}

unsigned long Program::hunToMillis(int hunst)
{
    return lrint(1000.0f*pow(2.0f, 0.01f*hunst));
}

int Program::slotAddr(int slot)
{
    return SLOTBASE+((slot-FIRSTSLOT)<<SLOTBITS);
}

void Program::save(int slot)
{
    if(slot < FIRSTSLOT || slot > LASTSLOT)
        return;
    
    int addr=slotAddr(slot);
    for(int i=0;i<MAXSTEPS;++i){
        EEPROM.write(addr++, (steps[i].stops >> 8) & 0xFF);
        EEPROM.write(addr++, steps[i].stops & 0xFF);
        EEPROM.write(addr++, steps[i].grade & 0xFF);
        for(int t=0;t<TEXTLEN;++t){
            EEPROM.write(addr++, steps[i].text[t]); 
        }
    }
}

bool Program::load(int slot)
{
    if(slot < FIRSTSLOT || slot > LASTSLOT)
        return false;
    
    int addr=slotAddr(slot);
    int tmp;
    for(int i=0;i<MAXSTEPS;++i){
        tmp=EEPROM.read(addr++) << 8;
        tmp|=EEPROM.read(addr++);
        steps[i].stops=tmp;
        steps[i].grade=EEPROM.read(addr++);
        for(int t=0;t<TEXTLEN;++t){
            steps[i].text[t]=EEPROM.read(addr++); 
        }
     
        // terminate string in memory
        steps[i].text[TEXTLEN]='\0';
    }
    return true;
}
