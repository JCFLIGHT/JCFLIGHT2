/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "stdint.h"

//CONVERTE STRING EM NÚMEROS INTEIROS OU FLOAT

int16_t ATO_Int(const char *Pointer)
{
    int16_t Signal = 1;
    int16_t Base = 0;
    int16_t IndexCounter = 0;

    while (Pointer[IndexCounter] == ' ')
    {
        IndexCounter++;
    }

    if (Pointer[IndexCounter] == '-' || Pointer[IndexCounter] == '+')
    {
        Signal = 1 - 2 * (Pointer[IndexCounter++] == '-');
    }

    while (Pointer[IndexCounter] >= '0' && Pointer[IndexCounter] <= '9')
    {
        if ((Base > (0x7fff / 10)) || (Base == (0x7fff / 10) && Pointer[IndexCounter] - '0' > 7))
        {
            if (Signal == 1)
            {
                return 0x7fff;
            }
            else
            {
                return (-0x7fff - 1);
            }
        }
        Base = 10 * Base + (Pointer[IndexCounter++] - '0');
    }
    return Base * Signal;
}