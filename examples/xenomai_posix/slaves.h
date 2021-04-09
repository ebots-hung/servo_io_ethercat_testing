/* Master 0, Slave 0, "EasyCAT 32+32 rev 1"
 * Vendor ID:       0x0000079a
 * Product code:    0x00defede
 * Revision number: 0x00005a01
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Byte0 */
    {0x0005, 0x02, 8}, /* Byte1 */
    {0x0005, 0x03, 8}, /* Byte2 */
    {0x0005, 0x04, 8}, /* Byte3 */
    {0x0005, 0x05, 8}, /* Byte4 */
    {0x0005, 0x06, 8}, /* Byte5 */
    {0x0005, 0x07, 8}, /* Byte6 */
    {0x0005, 0x08, 8}, /* Byte7 */
    {0x0005, 0x09, 8}, /* Byte8 */
    {0x0005, 0x0a, 8}, /* Byte9 */
    {0x0005, 0x0b, 8}, /* Byte10 */
    {0x0005, 0x0c, 8}, /* Byte11 */
    {0x0005, 0x0d, 8}, /* Byte12 */
    {0x0005, 0x0e, 8}, /* Byte13 */
    {0x0005, 0x0f, 8}, /* Byte14 */
    {0x0005, 0x10, 8}, /* Byte15 */
    {0x0005, 0x11, 8}, /* Byte16 */
    {0x0005, 0x12, 8}, /* Byte17 */
    {0x0005, 0x13, 8}, /* Byte18 */
    {0x0005, 0x14, 8}, /* Byte19 */
    {0x0005, 0x15, 8}, /* Byte20 */
    {0x0005, 0x16, 8}, /* Byte21 */
    {0x0005, 0x17, 8}, /* Byte22 */
    {0x0005, 0x18, 8}, /* Byte23 */
    {0x0005, 0x19, 8}, /* Byte24 */
    {0x0005, 0x1a, 8}, /* Byte25 */
    {0x0005, 0x1b, 8}, /* Byte26 */
    {0x0005, 0x1c, 8}, /* Byte27 */
    {0x0005, 0x1d, 8}, /* Byte28 */
    {0x0005, 0x1e, 8}, /* Byte29 */
    {0x0005, 0x1f, 8}, /* Byte30 */
    {0x0005, 0x20, 8}, /* Byte31 */
    {0x0006, 0x01, 8}, /* Byte0 */
    {0x0006, 0x02, 8}, /* Byte1 */
    {0x0006, 0x03, 8}, /* Byte2 */
    {0x0006, 0x04, 8}, /* Byte3 */
    {0x0006, 0x05, 8}, /* Byte4 */
    {0x0006, 0x06, 8}, /* Byte5 */
    {0x0006, 0x07, 8}, /* Byte6 */
    {0x0006, 0x08, 8}, /* Byte7 */
    {0x0006, 0x09, 8}, /* Byte8 */
    {0x0006, 0x0a, 8}, /* Byte9 */
    {0x0006, 0x0b, 8}, /* Byte10 */
    {0x0006, 0x0c, 8}, /* Byte11 */
    {0x0006, 0x0d, 8}, /* Byte12 */
    {0x0006, 0x0e, 8}, /* Byte13 */
    {0x0006, 0x0f, 8}, /* Byte14 */
    {0x0006, 0x10, 8}, /* Byte15 */
    {0x0006, 0x11, 8}, /* Byte16 */
    {0x0006, 0x12, 8}, /* Byte17 */
    {0x0006, 0x13, 8}, /* Byte18 */
    {0x0006, 0x14, 8}, /* Byte19 */
    {0x0006, 0x15, 8}, /* Byte20 */
    {0x0006, 0x16, 8}, /* Byte21 */
    {0x0006, 0x17, 8}, /* Byte22 */
    {0x0006, 0x18, 8}, /* Byte23 */
    {0x0006, 0x19, 8}, /* Byte24 */
    {0x0006, 0x1a, 8}, /* Byte25 */
    {0x0006, 0x1b, 8}, /* Byte26 */
    {0x0006, 0x1c, 8}, /* Byte27 */
    {0x0006, 0x1d, 8}, /* Byte28 */
    {0x0006, 0x1e, 8}, /* Byte29 */
    {0x0006, 0x1f, 8}, /* Byte30 */
    {0x0006, 0x20, 8}, /* Byte31 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 32, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1a00, 32, slave_0_pdo_entries + 32}, /* Inputs */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1, "EPOS4"
 * Vendor ID:       0x000000fb
 * Product code:    0x60500000
 * Revision number: 0x01610000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x6060, 0x00, 8},
    {0x6098, 0x00, 8},
    {0x6099, 0x01, 32},
    {0x6099, 0x02, 32},
    {0x609a, 0x00, 32},
    {0x30b1, 0x00, 32},
    {0x30b0, 0x00, 32},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 9, slave_1_pdo_entries + 0},
    {0x1a00, 5, slave_1_pdo_entries + 9},
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

