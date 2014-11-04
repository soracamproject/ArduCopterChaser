/** charset=UTF-8 **/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __BC_STATUS_H__
#define __BC_STATUS_H__

#include <inttypes.h>

// ***********************************************************************************
// 定義
// ***********************************************************************************
template <class T>
class BC_Status
{
public:
	// コンストラクタ
	BC_Status(uint8_t update_num):
		_status(0),
		_last(0),
		_count(0),
		_update_num(update_num)	// デフォルトは3回
	{}

	
	// 所定回数値の更新をする
	void update(T next_status);
	
	// 値の読み出し
	virtual T read() const { return _status; }
	
	// 更新に必要な回数を設定する（1回以上）
	void set_update_num(uint8_t update_num) { _update_num = max(1,update_num); }
	
private:
	T        _status;
	T        _last;
	uint8_t  _count;
	uint8_t  _update_num;
	
};


// ***********************************************************************************
// Typedef
// ***********************************************************************************
typedef BC_Status<int8_t> BC_Status_Int8;
typedef BC_Status<uint8_t> BC_Status_UInt8;
typedef BC_Status<int16_t> BC_Status_Int16;
typedef BC_Status<uint16_t> BC_Status_UInt16;
typedef BC_Status<int32_t> BC_Status_Int32;
typedef BC_Status<uint32_t> BC_Status_UInt32;
typedef BC_Status<float> BC_Status_Float;



// ***********************************************************************************
// 関数
// ***********************************************************************************
template <class T>
void BC_Status<T>::update(T next_status)
{
	if(next_status != _last){
		if(_update_num == 1){
			_status = next_status;
			_count = 0;
		} else {
			_count = 1;
		}
	} else {
		if(_count > 0 && ++_count >= _update_num){
			_status = next_status;
			_count = 0;
		}
	}
	_last = next_status;
}

#endif // __BC_STATUS_H__
