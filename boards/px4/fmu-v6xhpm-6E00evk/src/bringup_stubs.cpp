/****************************************************************************
 * HPM6E80 bring-up stubs
 *
 * USB VBUS sensing: there is no USB device port driver on this board yet,
 * but systemcmds/usb_connected links against this symbol.
 ****************************************************************************/

extern "C" int board_read_VBUS_state(void)
{
	return 0;
}
