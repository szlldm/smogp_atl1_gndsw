/*
 * Decoder and packet interpreter for Smog-P & ATL-1 pocketqube satellites
 * Copyright (C) 2019 szlldm
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MAIN_H
#define MAIN_H

double helper_ACK_INFO_RSSI_CONVERT(uint8_t u8)
{
	double tmp;

	tmp = u8;
	tmp /= 2.0;
	tmp -= 131.0;

	return tmp;
}

const double Ibat_calibr[4][2] = {
	{0.0091578069, 305.6611509242},
	{0.0092536381, 308.7354923643},
	{0.0092357733, 306.997781812},
	{0.0092097441, 308.1479044269},
};

double helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_battery_current(int panel, uint16_t adc)
{
	if (panel < 0) return NAN;
	if (panel > 3) return NAN;
	// 'panel' will be used for compensation by calibration values...

	const double Urefp = 2.044;
	const double Uref2p = 1.039;
	const double Urefm = 0.0;
	const double Rshunt = 0.034;
	const double lsb = (Urefp - Urefm) / 65536.0;

	double Ibat;
	//Ibat = (adc*lsb - Uref2p) / (100.0 * Rshunt);
	//kompenzáció értelmezése végett az egyenletet rendezni kell
	//Ibat = adc*(lsb/(100*Rshunt)) - (Uref2p/(100*Rshunt))
	Ibat = adc * Ibat_calibr[panel][0] - Ibat_calibr[panel][1];

	return Ibat/1000.0;
}

const double resist_correction[4][5][3] = {
	{
		{-0.0006651322, -0.0122560584, 0.751697002},
		{-0.0001612352, -0.054850064,  0.4956892858},
		{ 0.0005175386, -0.0151862359, 1.584156215},
		{-0.0005616397, -0.0035293076, 1.2312336719},
		{-0.0006161208,  0.0329681997, 2.3084111427},
	},
	{
		{-0.0008810698, -0.0005768639, 1.4462896472},
		{ 0.0002554855, -0.0497453659, 1.2655688326},
		{-0.0002372522,  0.0156908555, 2.3968181466},
		{-0.0005689415,  0.0082033445, 2.05142517512},
		{-0.000324599,   0.0315826991, 2.973868843},
	},
	{
		{-0.0005846387, -0.0212703774, 1.6960469081},
		{-0.0005227575, -0.029222141,  0.6464118577},
		{-0.0003972915,  0.0164286754, 2.2502388179},
		{ 0.0003169663, -0.0174945998, 1.7588053065},
		{-0.0004615105,  0.0385790667, 2.7849059873},
	},
	{
		{-0.0006318885, -0.0047932305, 1.6301837316},
		{-0.000244109,  -0.039820197,  1.3241749543},
		{-0.0005641744,  0.022177914,  2.4338965601},
		{-0.0004342717,  0.0057748108, 1.9869618945},
		{-0.0005652303,  0.0447351507, 3.0252586698},
	},
};

double helper_DOWNLINK_PCKT_TYPE_ATL_TELEMETRY_3_calc_temperature(int panel, uint16_t adc_ref, uint16_t adc, int channel)
{
	if (panel < 0) return NAN;
	if (panel > 3) return NAN;
	if (channel < 0) return NAN;
	if (channel > 4) return NAN;
	// 'panel' will be used for compensation by calibration values...

	const double Urefp = 2.046;
	const double Urefm = 0.0;
	const double R0 = 100.0;
	const double Rref = 72.9;
	const double A = 3.9083e-3;
	const double B = -5.775e-7;
	const double lsb = (Urefp - Urefm) / 65536.0;

	double Itemp, Rtemp;
	double Rt;
	double Tpt100;

	Itemp = (adc_ref * lsb) / (100.0 * Rref);
	Rtemp = ((adc * lsb) / (Itemp * 100.0));
	Rtemp = Rtemp - ((Rtemp*Rtemp*resist_correction[panel][channel][0]) + (Rtemp*resist_correction[panel][channel][1]) + (resist_correction[panel][channel][2]));
	Rt = Rref + Rtemp;
	Tpt100 = ((-1.0 * R0 * A) + sqrt((R0*R0 * A*A) - (4.0 * R0 * B * (R0 - Rt)))) / (2.0 * R0 * B);

	return Tpt100;
}

#endif


