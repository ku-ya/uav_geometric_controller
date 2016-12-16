#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void clear_string(char *string)
{
	string[0] = '\0';
}


void add_double_to_string(char *string, double value, char const *tag, bool last_value_in_string)
{
    char temp[100];

    sprintf(temp, "%s=%lf", tag, value);
    strcat(string, temp);
    if (last_value_in_string == false)
    {
        strcat(string, "&");
    }
}

double parse_string_to_double(char *string, char const *tag)
{
	char* string_copy = (char*)malloc(strlen(string) + 1);
	char* char_result;
	char* token;
	double result = 0.0;

	strcpy(string_copy, string);

	token = strtok(string_copy, "&");

	while(token)
	{
		char* equals_sign = strchr(token, '=');

		if(equals_sign)
		{
			*equals_sign = 0;

			if( 0 == strcmp(token, tag))
			{
				equals_sign++;
				char_result = (char*)malloc(strlen(equals_sign) + 1);
				strcpy(char_result, equals_sign);
				result = atof(char_result);
				free(char_result);
			}
		}
		token = strtok(0, "&");
	}
	free(string_copy);

	return result;
}
