/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "app/action.h"
#include "app/app.h"
#ifdef ENABLE_FMRADIO
	#include "app/fm.h"
#endif
#include "app/generic.h"
#include "app/main.h"
#include "app/search.h"
//#define ENABLE_SPECTRUM
#ifdef ENABLE_SPECTRUM
	#include "app/spectrum.h"
#endif
#include "audio.h"
#include "board.h"
#include "bsp/dp32g030/gpio.h"
#include "driver/bk4819.h"
#include "driver/gpio.h"
#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
	#include "driver/uart.h"
#endif
#include "dtmf.h"
#include "frequencies.h"
#ifdef ENABLE_SCAN_IGNORE_LIST
	#include "freq_ignore.h"
#endif
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/menu.h"
#include "ui/ui.h"

bool g_manual_scanning;

bool scanning_paused(void)
{
	if (g_scan_state_dir != SCAN_STATE_DIR_OFF &&
	    (g_scan_tick_10ms == 0 || g_scan_tick_10ms >= (400 / 10)))               // 400ms
	{
		return true;
	}

	if (g_eeprom.config.setting.dual_watch != DUAL_WATCH_OFF &&
	    (g_dual_watch_tick_10ms == 0 || g_dual_watch_tick_10ms >= (400 / 10)))   // 400ms
	{
		return true;
	}

	return (g_scan_state_dir == SCAN_STATE_DIR_OFF && g_eeprom.config.setting.dual_watch == DUAL_WATCH_OFF) ? true : false;
}

void toggle_chan_scanlist(void)
{	// toggle the selected channels scanlist setting

//	if (IS_FREQ_CHANNEL(g_tx_vfo->channel_save))    // TODO: include the VFO freq as a channel when scanning
	if (IS_NOAA_CHANNEL(g_tx_vfo->channel_save))
	{
		g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (g_current_display_screen != DISPLAY_MAIN     ||
		g_current_function == FUNCTION_TRANSMIT ||
		g_current_function == FUNCTION_PANADAPTER)
	{
		g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (!scanning_paused())
	{	// scanning isn't paused
		g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (g_tx_vfo->channel_attributes.scanlist1)
	{
		if (g_tx_vfo->channel_attributes.scanlist2)
			g_tx_vfo->channel_attributes.scanlist1 = 0;
		else
			g_tx_vfo->channel_attributes.scanlist2 = 1;
	}
	else
	{
		if (g_tx_vfo->channel_attributes.scanlist2)
			g_tx_vfo->channel_attributes.scanlist2 = 0;
		else
			g_tx_vfo->channel_attributes.scanlist1 = 1;
	}

	g_eeprom.config.channel_attributes[g_tx_vfo->channel_save] = g_tx_vfo->channel_attributes;

	SETTINGS_save_chan_attribs_name(g_tx_vfo->channel_save, g_tx_vfo);

	g_vfo_configure_mode = VFO_CONFIGURE;
	g_flag_reset_vfos    = true;
}

#ifdef ENABLE_COPY_CHAN_TO_VFO_TO_CHAN

	void MAIN_copy_mem_vfo_mem(void)
	{
		//const unsigned int vfo = get_RX_VFO();
		const unsigned int vfo = g_eeprom.config.setting.tx_vfo_num;

		if (g_css_scan_mode != CSS_SCAN_MODE_OFF || g_eeprom.config.setting.vfo_open == 0)
		{	// scanning or VFO disabled
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		}

		if (!scanning_paused())
		{	// RF scanning
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		}

		if (IS_USER_CHANNEL(g_eeprom.config.setting.indices.vfo[vfo].screen))
		{	// copy channel to VFO, then swap to the VFO

			const unsigned int channel = FREQ_CHANNEL_FIRST + g_vfo_info[vfo].channel_attributes.band;

			g_eeprom.config.setting.indices.vfo[vfo].screen = channel;
			g_vfo_info[vfo].channel_save        = channel;
			g_eeprom.config.setting.tx_vfo_num = vfo;

			RADIO_select_vfos();
			RADIO_apply_offset(g_tx_vfo, false);
			RADIO_ConfigureSquelch(g_tx_vfo);
//			RADIO_ConfigureTXPower(g_tx_vfo);
			RADIO_setup_registers(true);

			// find the first channel that contains this frequency
			g_tx_vfo->freq_in_channel = SETTINGS_find_channel(g_tx_vfo->freq_config_tx.frequency);

			SETTINGS_save_channel(g_tx_vfo->channel_save, g_eeprom.config.setting.tx_vfo_num, g_tx_vfo, 1);

			#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
				UART_printf("chan-vfo %u\r\n", g_tx_vfo->channel_save);
			#endif

			g_beep_to_play = BEEP_880HZ_60MS_TRIPLE_BEEP;
			//g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;

			g_update_status  = true;
			g_update_display = true;
		}
		else
		if (IS_NOT_NOAA_CHANNEL(g_eeprom.config.setting.indices.vfo[vfo].screen))
		{	// copy VFO to a channel

			// search the channels to see if the frequency is already present
			unsigned int chan = SETTINGS_find_channel(g_vfo_info[vfo].p_tx->frequency);
			if (chan > USER_CHANNEL_LAST)
			{	// not found - find next free channel to save too
				//for (chan = g_eeprom.config.setting.indices.vfo[vfo].screen; chan <= USER_CHANNEL_LAST; chan++)
				for (chan = 0; chan <= USER_CHANNEL_LAST; chan++)
					if (!RADIO_channel_valid(chan, false, vfo))
						break;
			}

			g_current_display_screen = DISPLAY_INVALID;
			GUI_SelectNextDisplay(DISPLAY_MENU);
			g_menu_cursor       = MENU_MEM_SAVE;
			g_in_sub_menu    = true;
			if (chan <= USER_CHANNEL_LAST)
			{
				#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
					UART_printf("vfo to mem %u\r\n", chan);
				#endif

				g_sub_menu_selection = chan;
				g_update_menu  = false;
				g_current_display_screen  = DISPLAY_MENU;
				g_update_display     = false;
				UI_DisplayMenu();
			}

			#ifdef ENABLE_VOICE
				g_another_voice_id = VOICE_ID_MENU;
			#endif

			g_beep_to_play = BEEP_880HZ_60MS_TRIPLE_BEEP;
		}
	}

#endif

void processFKeyFunction(const key_code_t Key)
{
	uint8_t vfo = g_eeprom.config.setting.tx_vfo_num;

	if (g_current_function == FUNCTION_TRANSMIT || g_current_display_screen == DISPLAY_MENU)
	{
		g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	switch (Key)
	{
		case KEY_0:   // FM

			if (g_fkey_pressed)
			{
				if (++g_tx_vfo->channel.mod_mode >= MOD_MODE_LEN)
					g_tx_vfo->channel.mod_mode = 0;

				AUDIO_set_mod_mode(g_tx_vfo->channel.mod_mode);

				if (IS_FREQ_CHANNEL(g_tx_vfo->channel_save))
					if (g_scan_state_dir == SCAN_STATE_DIR_OFF)
						g_request_save_vfo = true;

				g_request_display_screen = DISPLAY_MAIN;
			}
			else
			{

				#ifdef ENABLE_FMRADIO
					if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
						APP_stop_scan();

					ACTION_FM();
				#else


					// TODO: make use of this function key


				#endif
			}

			break;

		case KEY_1:   // BAND

			if (!IS_FREQ_CHANNEL(g_tx_vfo->channel_save))
			{
				g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
				return;
			}

			APP_stop_scan();

			{
				unsigned int band = g_tx_vfo->channel_attributes.band;

				#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
					UART_printf("band %u\r\n", band);
				#endif

				if (++band > BAND7_470MHz)
					band = BAND1_50MHz;
				if (!g_eeprom.config.setting.enable_350 && band == BAND5_350MHz)
					band = BAND6_400MHz;      // jump to next band
				g_tx_vfo->channel_attributes.band = band;

				g_eeprom.config.setting.indices.vfo[vfo].screen    = FREQ_CHANNEL_FIRST + band;
				g_eeprom.config.setting.indices.vfo[vfo].frequency = FREQ_CHANNEL_FIRST + band;
			}

			g_request_save_vfo   = true;
			g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;

			g_request_display_screen = DISPLAY_MAIN;

			break;

		case KEY_2:   // A/B

			APP_stop_scan();

			if (g_eeprom.config.setting.cross_vfo == CROSS_BAND_CHAN_A)
				g_eeprom.config.setting.cross_vfo = CROSS_BAND_CHAN_B;
			else
			if (g_eeprom.config.setting.cross_vfo == CROSS_BAND_CHAN_B)
				g_eeprom.config.setting.cross_vfo = CROSS_BAND_CHAN_A;
			else
			if (g_eeprom.config.setting.dual_watch == DUAL_WATCH_CHAN_A)
				g_eeprom.config.setting.dual_watch = DUAL_WATCH_CHAN_B;
			else
			if (g_eeprom.config.setting.dual_watch == DUAL_WATCH_CHAN_B)
				g_eeprom.config.setting.dual_watch = DUAL_WATCH_CHAN_A;
			else
				g_eeprom.config.setting.tx_vfo_num = (vfo + 1) & 1u;

			g_request_save_settings = 1;
			g_flag_reconfigure_vfos = true;

			g_request_display_screen = DISPLAY_MAIN;
			break;

		case KEY_3:   // VFO/MR

			APP_stop_scan();

			if (g_eeprom.config.setting.vfo_open > 0 && IS_NOT_NOAA_CHANNEL(g_tx_vfo->channel_save))
			{
				uint8_t Channel;

				if (IS_USER_CHANNEL(g_tx_vfo->channel_save))
				{	// swap to frequency mode
					g_eeprom.config.setting.indices.vfo[vfo].screen = g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].frequency;

					#ifdef ENABLE_VOICE
						g_another_voice_id = VOICE_ID_FREQUENCY_MODE;
					#endif

					g_request_save_vfo   = true;
					g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;
					break;
				}

				Channel = RADIO_FindNextChannel(g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].user, 1, false, 0);
				if (Channel != 0xFF)
				{	// swap to channel mode
					g_eeprom.config.setting.indices.vfo[vfo].screen= Channel;

					#ifdef ENABLE_VOICE
						AUDIO_SetVoiceID(0, VOICE_ID_CHANNEL_MODE);
						AUDIO_SetDigitVoice(1, Channel + 1);
						g_another_voice_id = (voice_id_t)0xFE;
					#endif

					g_request_save_vfo   = true;
					g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;
					break;
				}
			}

			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;

			break;

		case KEY_4:    // FC

			APP_stop_scan();

			g_search_flag_start_scan  = true;
			g_search_single_frequency = false;
			g_backup_cross_vfo  = g_eeprom.config.setting.cross_vfo;
			g_eeprom.config.setting.cross_vfo = CROSS_BAND_OFF;
			break;

		case KEY_5:    // NOAA

			#ifdef ENABLE_PANADAPTER
				if (g_fkey_pressed)
				{
					g_fkey_pressed = false;
					g_eeprom.config.setting.panadapter = (g_eeprom.config.setting.panadapter + 1) & 1u;
					g_request_save_settings = true;
					break;
				}
			#endif

			#ifdef ENABLE_NOAA

				APP_stop_scan();

				if (IS_NOT_NOAA_CHANNEL(g_tx_vfo->channel_save))
				{
					g_eeprom.config.setting.indices.vfo[vfo].screen = g_eeprom.config.setting.indices.noaa_channel[g_eeprom.config.setting.tx_vfo_num];
				}
				else
				{
					g_eeprom.config.setting.indices.vfo[vfo].screen = g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].frequency;

					#ifdef ENABLE_VOICE
						g_another_voice_id = VOICE_ID_FREQUENCY_MODE;
					#endif
				}
				g_request_save_vfo   = true;
				g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;

			#else
				toggle_chan_scanlist();
			#endif
#ifdef	ENABLE_SPECTRUM
					APP_RunSpectrum();
					g_request_display_screen = DISPLAY_MAIN;

#endif

			break;

		case KEY_6:    // H/M/L

			if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
			{
				g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
				return;
			}

			ACTION_Power();
			break;

		case KEY_7:    // VOX

			#ifdef ENABLE_VOX
				APP_stop_scan();
				ACTION_Vox();
			#elif defined(ENABLE_NOAA)
				toggle_chan_scanlist();
			#endif

			break;

		case KEY_8:    // R

			if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
			{
				g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
				return;
			}

			g_tx_vfo->channel.frequency_reverse = g_tx_vfo->channel.frequency_reverse == false;
			g_request_save_channel = 1;

			break;

		case KEY_9:    // CALL

			if (!RADIO_channel_valid(g_eeprom.config.setting.call1, false, 0))
			{
				g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
				return;
			}

			// swap to the CALL channel

			APP_stop_scan();

			g_eeprom.config.setting.indices.vfo[vfo].user   = g_eeprom.config.setting.call1;
			g_eeprom.config.setting.indices.vfo[vfo].screen = g_eeprom.config.setting.call1;

			#ifdef ENABLE_VOICE
				AUDIO_SetVoiceID(0, VOICE_ID_CHANNEL_MODE);
				AUDIO_SetDigitVoice(1, 1 + g_eeprom.config.setting.call1);
				g_another_voice_id = (voice_id_t)0xFE;
			#endif

			g_request_save_vfo   = true;
			g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;

			break;

		default:
//			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			break;
	}
}

void MAIN_Key_DIGITS(key_code_t Key, bool key_pressed, bool key_held)
{
	const uint8_t vfo = g_eeprom.config.setting.tx_vfo_num;

	g_key_input_count_down = key_input_timeout_500ms;

	#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
//		UART_printf("key0 %u\r\n", Key);
	#endif

	if (key_held)
	{	// key held down

		if (key_pressed)
		{
			if (g_current_display_screen == DISPLAY_MAIN)
			{
				if (g_input_box_index > 0)
				{	// clear the user box
					g_input_box_index        = 0;
					g_request_display_screen = DISPLAY_MAIN;
				}

				processFKeyFunction(Key);

				g_fkey_pressed  = false;
				g_update_status = true;
			}
		}

		return;
	}

	if (key_pressed && !key_held)
	{	// key just pressed
		g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;
		return;                                    // don't use the key till it's released
	}

	if (g_fkey_pressed)
	{	// F-key was first pressed

		processFKeyFunction(Key);

		g_fkey_pressed  = false;
		g_update_status = true;
		return;
	}

	if (g_current_function == FUNCTION_TRANSMIT)
		return;

	if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
	{
		g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	// add the digit to the channel/frequency input box

	INPUTBOX_append(Key);

//	UI_DisplayMain();

	g_request_display_screen = DISPLAY_MAIN;

	if (IS_USER_CHANNEL(g_tx_vfo->channel_save))
	{	// user is entering channel number

		const unsigned int chan = ((g_input_box[0] * 100) + (g_input_box[1] * 10) + g_input_box[2]) - 1;

	#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
//		UART_printf("key2 %u %u\r\n", chan, g_input_box_index);
	#endif

		if (g_input_box_index < 3)
		{
			#ifdef ENABLE_VOICE
				g_another_voice_id = (voice_id_t)Key;
			#endif
			return;
		}

		g_input_box_index = 0;

		if (!RADIO_channel_valid(chan, false, 0))
		{
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		}

		#ifdef ENABLE_VOICE
			g_another_voice_id = (voice_id_t)Key;
		#endif

		g_eeprom.config.setting.indices.vfo[vfo].user   = chan;
		g_eeprom.config.setting.indices.vfo[vfo].screen = chan;
		g_request_save_vfo            = true;
		g_vfo_configure_mode          = VFO_CONFIGURE_RELOAD;

		g_update_display = true;
		return;
	}

	if (IS_FREQ_CHANNEL(g_tx_vfo->channel_save))
	{	// user is entering a frequency

		uint32_t freq;

		NUMBER_Get(g_input_box, &freq);

	#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
//		UART_printf("key3 %u %u\r\n", freq, g_input_box_index);
	#endif

//		if (g_input_box_index < 6)
//		if (g_input_box_index < 7)
		if (g_input_box_index < 8)
		{
			#ifdef ENABLE_VOICE
				g_another_voice_id = (voice_id_t)Key;
			#endif
			return;
		}

		g_input_box_index = 0;

		// clamp the frequency entered to some valid value
		if (freq < FREQ_BAND_TABLE[0].lower)
			freq = FREQ_BAND_TABLE[0].lower;
		else
		if (freq >= BX4819_BAND1.upper && freq < BX4819_BAND2.lower)
		{
			const uint32_t center = (BX4819_BAND1.upper + BX4819_BAND2.lower) / 2;
			freq = (freq < center) ? BX4819_BAND1.upper : BX4819_BAND2.lower;
		}
		else
		if (freq > FREQ_BAND_TABLE[ARRAY_SIZE(FREQ_BAND_TABLE) - 1].upper)
			freq = FREQ_BAND_TABLE[ARRAY_SIZE(FREQ_BAND_TABLE) - 1].upper;

		{
			const frequency_band_t band = FREQUENCY_GetBand(freq);

			#ifdef ENABLE_VOICE
				g_another_voice_id = (voice_id_t)Key;
			#endif

			if (g_tx_vfo->channel_attributes.band != band)
			{
				g_tx_vfo->channel_attributes.band = band;
				g_eeprom.config.setting.indices.vfo[vfo].screen    = band + FREQ_CHANNEL_FIRST;
				g_eeprom.config.setting.indices.vfo[vfo].frequency = band + FREQ_CHANNEL_FIRST;

				SETTINGS_save_vfo_indices();

				RADIO_configure_channel(vfo, VFO_CONFIGURE_RELOAD);
			}

			#if 0
				freq += g_tx_vfo->step_freq / 2; // for rounding to nearest step size
				freq = FREQUENCY_floor_to_step(freq, g_tx_vfo->step_freq, FREQ_BAND_TABLE[g_tx_vfo->channel_attributes.band].lower, FREQ_BAND_TABLE[g_tx_vfo->channel_attributes.band].upper);
			#endif

			if (freq >= BX4819_BAND1.upper && freq < BX4819_BAND2.lower)
			{	// clamp the frequency to the limit
				const uint32_t center = (BX4819_BAND1.upper + BX4819_BAND2.lower) / 2;
				freq = (freq < center) ? BX4819_BAND1.upper - g_tx_vfo->step_freq : BX4819_BAND2.lower;
			}

			g_tx_vfo->freq_config_rx.frequency = freq;
			g_tx_vfo->freq_config_tx.frequency = freq;

			// find the first channel that contains this frequency
			g_tx_vfo->freq_in_channel = SETTINGS_find_channel(freq);

			g_request_save_channel = 1;
			g_vfo_configure_mode   = VFO_CONFIGURE;

			g_update_display = true;
			return;
		}

	}
	#ifdef ENABLE_NOAA
		else
		if (IS_NOAA_CHANNEL(g_tx_vfo->channel_save))
		{	// user is entering NOAA channel

			uint8_t Channel;

			if (g_input_box_index != 2)
			{
				#ifdef ENABLE_VOICE
					g_another_voice_id = (voice_id_t)Key;
				#endif
//				g_request_display_screen = DISPLAY_MAIN;
				return;
			}

			g_input_box_index = 0;

			Channel = (g_input_box[0] * 10) + g_input_box[1];
			if (Channel >= 1 && Channel <= ARRAY_SIZE(NOAA_FREQUENCY_TABLE))
			{
				Channel += NOAA_CHANNEL_FIRST;
				#ifdef ENABLE_VOICE
					g_another_voice_id = (voice_id_t)Key;
				#endif
				g_eeprom.config.setting.indices.noaa_channel[vfo] = Channel;
				g_eeprom.config.setting.indices.vfo[vfo].screen = Channel;
				g_request_save_vfo   = true;
				g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;
				g_update_display     = true;
				return;
			}
		}
	#endif

	g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;

	g_update_display = true;
//	g_request_display_screen = DISPLAY_MAIN;
}

void MAIN_Key_EXIT(bool key_pressed, bool key_held)
{
	if (key_pressed && !key_held)
	{	// key just pressed

		g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;

		if (g_dtmf_call_state != DTMF_CALL_STATE_NONE && g_current_function != FUNCTION_TRANSMIT)
		{	// clear CALL mode being displayed
			g_dtmf_call_state = DTMF_CALL_STATE_NONE;
			g_update_display  = true;
			return;
		}

		#ifdef ENABLE_FMRADIO
			if (!g_fm_radio_mode)
		#endif
		{
			if (g_scan_state_dir == SCAN_STATE_DIR_OFF)
			{
				if (g_input_box_index == 0)
					return;
				g_input_box[--g_input_box_index] = 10;

				g_key_input_count_down = key_input_timeout_500ms;

				#ifdef ENABLE_VOICE
					if (g_input_box_index == 0)
						g_another_voice_id = VOICE_ID_CANCEL;
				#endif
			}
			else
			{
				APP_stop_scan();

				#ifdef ENABLE_VOICE
					g_another_voice_id = VOICE_ID_SCANNING_STOP;
				#endif
			}

			g_request_display_screen = DISPLAY_MAIN;
			return;
		}

		#ifdef ENABLE_FMRADIO
			ACTION_FM();
		#endif

		return;
	}

	if (!key_held)
		return;

	if (key_pressed)
		return;

	#ifdef ENABLE_FMRADIO
		if (g_fm_radio_mode)
		{
			ACTION_FM();
			return;
		}
	#endif

	if (g_input_box_index > 0 || g_dtmf_input_box_index > 0 || g_dtmf_input_mode)
	{	// cancel key input mode (channel/frequency entry)
		g_dtmf_input_mode       = false;
		g_dtmf_input_box_index  = 0;
		memset(g_dtmf_string, 0, sizeof(g_dtmf_string));
		g_input_box_index        = 0;
		g_request_display_screen = DISPLAY_MAIN;
		g_beep_to_play           = BEEP_1KHZ_60MS_OPTIONAL;
		return;
	}

	if (g_flash_light_state != FLASHLIGHT_OFF && g_flash_light_state != FLASHLIGHT_SOS)
	{	// the the flash light off
		g_flash_light_state = FLASHLIGHT_OFF;
		GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_FLASHLIGHT);
		return;
	}
}

void MAIN_Key_MENU(const bool key_pressed, const bool key_held)
{
	if (key_pressed && !key_held)
	{	// key just pressed
		AUDIO_PlayBeep(BEEP_1KHZ_60MS_OPTIONAL);
	}

	if (key_held)
	{	// menu key held down (long press)

		if (key_pressed)
		{	// long press MENU key

			g_fkey_pressed = false;

			if (g_current_display_screen == DISPLAY_MAIN)
			{
				if (g_input_box_index > 0)
				{	// delete any inputted chars
					g_input_box_index        = 0;
					g_request_display_screen = DISPLAY_MAIN;
				}

				g_fkey_pressed  = false;
				g_update_status = true;

				#ifdef ENABLE_SCAN_IGNORE_LIST

					if (g_scan_state_dir == SCAN_STATE_DIR_OFF &&
					    g_css_scan_mode == CSS_SCAN_MODE_OFF &&
					    FI_freq_ignored(g_rx_vfo->p_rx->frequency) >= 0)
					{	// remove the frequency from the ignore list
						FI_sub_freq_ignored(g_rx_vfo->p_rx->frequency);
						g_update_display = true;
					}
					#ifdef ENABLE_COPY_CHAN_TO_VFO_TO_CHAN
						else
							MAIN_copy_mem_vfo_mem();
					#endif

				#elif defined(ENABLE_COPY_CHAN_TO_VFO_TO_CHAN)

					MAIN_copy_mem_vfo_mem();

				#endif
			}
		}

		return;
	}

	if (!key_pressed && !g_dtmf_input_mode)
	{	// menu key released

		const bool flag = (g_input_box_index == 0);

		g_input_box_index = 0;

		if (flag)
		{
			g_update_menu = true;
			g_request_display_screen = DISPLAY_MENU;

			#ifdef ENABLE_VOICE
				g_another_voice_id   = VOICE_ID_MENU;
			#endif
		}
		else
		{
			g_request_display_screen = DISPLAY_MAIN;
		}
	}
}

void MAIN_Key_STAR(bool key_pressed, bool key_held)
{
	if (g_input_box_index > 0)
	{	// entering a channel, frequency or DTMF string
		if (!key_held && key_pressed)
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (key_held && !g_fkey_pressed)
	{	// long press .. toggle scanning

		if (!key_pressed)
			return;          // released

		ACTION_Scan(false);

//		g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;
		return;
	}

	if (key_pressed)
	{	// just pressed
		g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;
		return;
	}

	if (g_current_function == FUNCTION_TRANSMIT)
		return;

	if (!g_fkey_pressed)
	{	// pressed without the F-key

		if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
		{	// RF scanning

			#ifdef ENABLE_SCAN_IGNORE_LIST
				if (scanning_paused())
				{
					if (!FI_add_freq_ignored(g_rx_vfo->freq_config_rx.frequency))
						g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;  // not added for some reason

					// immediately continue the scan
					g_scan_tick_10ms = 0;
					g_scan_pause_time_mode = false;
					g_squelch_open         = false;
					g_rx_reception_mode    = RX_MODE_NONE;
					FUNCTION_Select(FUNCTION_FOREGROUND);
				}
			#endif

			return;
		}

		if (g_scan_state_dir == SCAN_STATE_DIR_OFF && IS_NOT_NOAA_CHANNEL(g_tx_vfo->channel_save))
		{	// start entering a DTMF string

			memcpy( g_dtmf_input_box, g_dtmf_string,
				(sizeof(g_dtmf_input_box) <= (sizeof(g_dtmf_string) - 1)) ? sizeof(g_dtmf_input_box) : sizeof(g_dtmf_string) - 1);

			g_dtmf_input_box_index  = 0;
			g_dtmf_input_mode       = true;

			g_key_input_count_down = key_input_timeout_500ms;

			g_request_display_screen = DISPLAY_MAIN;
		}
	}
	else
	{	// with the F-key
		g_fkey_pressed = false;

		if (g_scan_state_dir != SCAN_STATE_DIR_OFF)
			return;	// RF scanning

		if (IS_NOAA_CHANNEL(g_tx_vfo->channel_save))
		{
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		}

		// scan the CTCSS/DCS code
		g_search_flag_start_scan  = true;
		g_search_single_frequency = true;
		g_backup_cross_vfo  = g_eeprom.config.setting.cross_vfo;
		g_eeprom.config.setting.cross_vfo = CROSS_BAND_OFF;
	}

	g_ptt_was_released = true;

	g_update_status = true;
}

void MAIN_Key_UP_DOWN(bool key_pressed, bool key_held, scan_state_dir_t direction)
{
	#ifdef ENABLE_SQ_OPEN_WITH_UP_DN_BUTTS
		static bool monitor_was_enabled = false;
	#endif

	uint8_t Channel = g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].screen;

	if (key_pressed && !key_held)
	{	// key just pressed
		g_beep_to_play    = BEEP_1KHZ_60MS_OPTIONAL;
		g_manual_scanning = false;
	}

	if (!key_pressed)
	{	// key released

		if (g_scan_state_dir == SCAN_STATE_DIR_OFF && (Channel <= USER_CHANNEL_LAST || IS_FREQ_CHANNEL(Channel)))
		{
			#ifdef ENABLE_SQ_OPEN_WITH_UP_DN_BUTTS
				if (key_held && !monitor_was_enabled)
				{	// re-enable the squelch

					g_monitor_enabled = false;
//					GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);
					APP_start_listening();
				}
			#endif

			g_tx_vfo->freq_config_tx.frequency = g_tx_vfo->freq_config_rx.frequency;

			// find the first channel that contains this frequency
			g_tx_vfo->freq_in_channel = SETTINGS_find_channel(g_tx_vfo->freq_config_rx.frequency);

			SETTINGS_save_channel(g_tx_vfo->channel_save, g_eeprom.config.setting.tx_vfo_num, g_tx_vfo, 1);

			RADIO_apply_offset(g_tx_vfo, true);

			#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
//				UART_printf("save chan %u\r\n", g_rx_vfo->channel_save);
			#endif
		}

		g_manual_scanning = false;
	}

	if (key_held || !key_pressed)
	{	// long press

		if (g_input_box_index > 0)
			return;

		if (!key_pressed)
		{
			if (!key_held)
				return;

			if (IS_FREQ_CHANNEL(Channel))
				return;

			#ifdef ENABLE_VOICE
				AUDIO_SetDigitVoice(0, g_tx_vfo->channel_save + 1);
				g_another_voice_id = (voice_id_t)0xFE;
			#endif

			return;
		}
	}
	else
	{
		if (g_input_box_index > 0)
		{
			g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		}

//		g_beep_to_play = BEEP_1KHZ_60MS_OPTIONAL;
	}

	if (g_scan_state_dir == SCAN_STATE_DIR_OFF)
	{	// not RF scanning

		if (IS_NOT_NOAA_CHANNEL(Channel))
		{
			uint8_t Next;

			if (IS_FREQ_CHANNEL(Channel))
			{	// frequency mode

				uint32_t               freq = g_tx_vfo->freq_config_rx.frequency;
				const frequency_band_t band = FREQUENCY_GetBand(freq);

				if (key_pressed && !key_held)
				{	// just pressed

					g_scan_initial_upper     = FREQ_BAND_TABLE[band].upper;
					g_scan_initial_lower     = FREQ_BAND_TABLE[band].lower;
					g_scan_initial_step_size = g_tx_vfo->step_freq;

					#ifdef ENABLE_SCAN_RANGES
						//if (g_eeprom.config.setting.scan_ranges_enable)
						//{
						//	FREQUENCY_scan_range(freq, &g_scan_initial_lower, &g_scan_initial_upper, &g_scan_initial_step_size);
						//	freq = FREQUENCY_floor_to_step(freq, g_scan_initial_step_size, g_scan_initial_lower, g_scan_initial_upper);
						//}
					#endif
				}

				freq += g_scan_initial_step_size * direction;

				// wrap-a-round
//				if (key_held)
				{
					while (freq >= g_scan_initial_upper)
						freq -= g_scan_initial_upper - g_scan_initial_lower;
					while (freq < g_scan_initial_lower)
						freq += g_scan_initial_upper - g_scan_initial_lower;
				}

				// round
				#ifdef ENABLE_SCAN_RANGES
					//if (key_held)
					//	//freq = g_scan_initial_lower + ((((freq - g_scan_initial_lower) + (g_scan_initial_step_size / 2)) / g_scan_initial_step_size) * g_scan_initial_step_size);
					//	freq = FREQUENCY_floor_to_step(freq, g_scan_initial_step_size, g_scan_initial_lower, g_scan_initial_upper);
				#endif

				if (band == BAND2_108MHz)   // cope with the 8.333kHz step size
					freq = FREQUENCY_floor_to_step(freq, g_scan_initial_step_size, g_scan_initial_lower, g_scan_initial_upper);

				if (FREQUENCY_rx_freq_check(freq) < 0)
				{	// frequency not allowed
					g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
					return;
				}

				g_tx_vfo->freq_config_rx.frequency = freq;

				// find the first channel that contains this frequency .. currently takes too long
				//
				//if (!key_held && key_pressed)
				//	g_tx_vfo->freq_in_channel = SETTINGS_find_channel(freq);
				//else
				//if (key_held && key_pressed)
					g_tx_vfo->freq_in_channel = 0xff;

				#if 0
					RADIO_apply_offset(g_tx_vfo, false);
					RADIO_ConfigureSquelch(g_tx_vfo);
//					RADIO_ConfigureTXPower(g_tx_vfo);

					// original slow method
					g_request_save_channel = 1;
				#else
					// don't need to go through all the other stuff
					// lets speed things up by simply setting the VCO/PLL frequency
					// and the RF filter path (LNA and PA)

					#ifdef ENABLE_SQ_OPEN_WITH_UP_DN_BUTTS
						if (!key_held && key_pressed)
							monitor_was_enabled = g_monitor_enabled;

						if (key_held && key_pressed && !monitor_was_enabled)
						{	// open the squelch if the user holds the key down
							g_manual_scanning = true;
							g_monitor_enabled = true;
							GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);
						}
					#endif

					BK4819_set_rf_frequency(freq, true);  // set the VCO/PLL
					BK4819_set_rf_filter_path(freq);      // set the proper LNA/PA filter path

					RADIO_apply_offset(g_tx_vfo, false);
					RADIO_ConfigureSquelch(g_tx_vfo);
//					RADIO_ConfigureTXPower(g_tx_vfo);
				#endif

				return;
			}

			// channel mode

			g_tx_vfo->freq_in_channel = 0xff;

			Next = RADIO_FindNextChannel(Channel + direction, direction, false, 0);
			if (Next == 0xFF)
				return;

			if (Channel == Next)
				return;

			#ifdef ENABLE_SQ_OPEN_WITH_UP_DN_BUTTS
				if (!key_held && key_pressed)
					monitor_was_enabled = g_monitor_enabled;

				if (key_held && key_pressed && !monitor_was_enabled)
				{	// open the squelch if the user holds the key down
					g_manual_scanning = true;
					g_monitor_enabled = true;
					GPIO_SetBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);
//					APP_start_listening();
				}
			#endif

			g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].user   = Next;
			g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].screen = Next;

			if (!key_held)
			{
				#ifdef ENABLE_VOICE
					AUDIO_SetDigitVoice(0, Next + 1);
					g_another_voice_id = (voice_id_t)0xFE;
				#endif
			}
		}
		#ifdef ENABLE_NOAA
			else
			{
				Channel = NOAA_CHANNEL_FIRST + NUMBER_AddWithWraparound(g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].screen - NOAA_CHANNEL_FIRST, direction, 0, 9);
				g_eeprom.config.setting.indices.noaa_channel[g_eeprom.config.setting.tx_vfo_num] = Channel;
				g_eeprom.config.setting.indices.vfo[g_eeprom.config.setting.tx_vfo_num].screen   = Channel;
			}
		#endif

		if (!key_held && key_pressed)    // save when the user releases the button - save a LOT of eeprom wear
			g_request_save_vfo = true;

		g_vfo_configure_mode = VFO_CONFIGURE_RELOAD;

		return;
	}

	GPIO_ClearBit(&GPIOC->DATA, GPIOC_PIN_SPEAKER);

	// jump to the next channel
	APP_channel_next(false, direction);

	// go NOW
	g_scan_tick_10ms = 0;
	g_scan_pause_time_mode = false;
	g_squelch_open         = false;
	g_rx_reception_mode    = RX_MODE_NONE;
	FUNCTION_Select(FUNCTION_FOREGROUND);

	g_ptt_was_released = true;
}

void MAIN_process_key(key_code_t key, bool key_pressed, bool key_held)
{
	#if defined(ENABLE_UART) && defined(ENABLE_UART_DEBUG)
//		UART_printf(" main 1 key %2u %u %u %u\r\n", key, key_pressed, key_held);
	#endif

	#ifdef ENABLE_FMRADIO
		if (g_fm_radio_mode && key != KEY_PTT && key != KEY_EXIT)
		{
			if (g_current_display_screen == DISPLAY_FM)
			{
				if (!key_held && key_pressed)
					g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
				return;
			}
		}
	#endif

	if (g_dtmf_input_mode)
	{
		const char Character = DTMF_GetCharacter(key);
		if (Character != 0xFF)
		{	// add key to DTMF string
			if (key_pressed && !key_held)
			{
				DTMF_Append(Character);
				g_key_input_count_down   = key_input_timeout_500ms;
				g_beep_to_play           = BEEP_1KHZ_60MS_OPTIONAL;
				g_request_display_screen = DISPLAY_MAIN;
				g_ptt_was_released       = true;
			}
			return;
		}
	}

	switch (key)
	{
		case KEY_0:
		case KEY_1:
		case KEY_2:
		case KEY_3:
		case KEY_4:
		case KEY_5:
		case KEY_6:
		case KEY_7:
		case KEY_8:
		case KEY_9:
			MAIN_Key_DIGITS(key, key_pressed, key_held);
			break;
		case KEY_MENU:
			MAIN_Key_MENU(key_pressed, key_held);
			break;
		case KEY_UP:
			MAIN_Key_UP_DOWN(key_pressed, key_held, SCAN_STATE_DIR_FORWARD);
			break;
		case KEY_DOWN:
			MAIN_Key_UP_DOWN(key_pressed, key_held, SCAN_STATE_DIR_REVERSE);
			break;
		case KEY_EXIT:
			MAIN_Key_EXIT(key_pressed, key_held);
			break;
		case KEY_STAR:
			MAIN_Key_STAR(key_pressed, key_held);
			break;
		case KEY_F:
			GENERIC_Key_F(key_pressed, key_held);
			break;
		case KEY_PTT:
			GENERIC_Key_PTT(key_pressed);
			break;
		default:
			if (!key_held && key_pressed)
				g_beep_to_play = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			break;
	}
}
