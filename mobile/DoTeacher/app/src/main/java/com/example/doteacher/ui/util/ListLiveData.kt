package hustle.com.util.custom

import androidx.lifecycle.MutableLiveData

class ListLiveData<T> : MutableLiveData<MutableList<T>>() {
  private var temp: MutableList<T> = mutableListOf()

  init {
    value = temp
  }

  override fun getValue() = super.getValue()!!
  override fun setValue(value: MutableList<T>?) = super.setValue(value)


  val size: Int get() = value.size
  operator fun get(idx: Int) =
    if (size > idx) value[idx] else throw ArrayIndexOutOfBoundsException("Index $idx Size $size")

  fun data(): List<T> {
    return temp
  }

  fun add(item: T) {
    temp.add(item)
    value = temp
  }
  fun addAll(list: List<T>) {
    temp.addAll(list)
    value = temp
  }
  fun clear() {
    temp.clear()
    value = temp
  }

}