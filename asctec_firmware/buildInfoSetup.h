

void generateBuildInfo(void);

struct __attribute__((packed)) BUILD_INFO
{
	unsigned short version_major;
	unsigned short version_minor;
	unsigned int build_date;
	unsigned int build_number;
	unsigned int configuration;
	unsigned short svn_revision;
	unsigned char svn_modified;
	char svn_url[64];
};

extern struct BUILD_INFO buildInfo;

#define BUILDINFO_EEPROM_FOUND       0x04000000
#define BUILDINFO_EEPROM_LUMIX_ZOOM  0x08000000
#define BUILDINFO_EEPROM_OPT_VIDEOSW 0x10000000
#define BUILDINFO_EEPROM_OPT_SERIAL  0x20000000
#define BUILDINFO_EEPROM_SONY_NEX5   0x40000000
#define BUILDINFO_SC16_FOUND 		 0x80000000
