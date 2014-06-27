TFT_ILI9340
===========

A hyper fast SPI driver for Ilitek ILI9340 TFT 2.2" displays
	Features:
	- Very FAST!, expecially with Teensy 3.x where uses native SPI.
	- It uses just 4 or 5 wires.
	- Compatible at command level with Adafruit display series so it's easy to adapt existing code.
	- It uses the standard Adafruit_GFX Library (you need to install). 
	
	Background:
	Adafruit it's a great company, always very nice with developers and released a library for
	all his products completely free downloadable. However I don't like some of their libraries
	since are slow and not optimized for all popular MCU. I got a display from Ebay
	http://www.ebay.com/itm/281304733556
	and I'm not using the Adafruit one since was not available when I ordered but it looks
	exact the same and works in the same way so I've tried the related library first that
	was slow and not optimized, it works but can be much better.
	Paul Stoffregen and Adafruit maded together a great work for the ST7735 that it's a popular
	display and works fast with almost all popular MCU so I grab some of the routines and applied
	to this one, optimized some of the code and result it was really nice.
	
	
	Code Optimizations:
	The purpose of this library it's SPEED. I have tried to use hardware optimized calls
	where was possible and results are quite good for most applications, actually nly filled circles
    are still a bit slow. Many SPI call has been optimized by reduce un-needed triggers to RS and CS
	lines. Of course it can be improved so feel free to add suggestions.
	-------------------------------------------------------------------------------
    Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa.    

    ILI9340C Library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ILI9340C Library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    This file needs the following Libraries:
 
    Adafruit_GFX by Adafruit:
    https://github.com/adafruit/Adafruit-GFX-Library
	Remember to update GFX library often to have more features with this library!
	From this version I'm using my version of Adafruit_GFX library:
	https://github.com/sumotoy/Adafruit-GFX-Library
	It has faster char rendering and some small little optimizations but you can
	choose one of the two freely since are both fully compatible.
	''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
	Special Thanks:
	Thanks Adafruit for his Adafruit_GFX!
	Thanks to Paul Stoffregen for his beautiful Teensy3 and native SPI.
	
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Version:
	0.5b1: First one and working.
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	BugList of the current version:
	
	- Actually no scroll commands (only in release will be included).
