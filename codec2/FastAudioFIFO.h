#pragma once

/*
Fast 2048 int16_t circular buffer
Eng. Deulis Antonio Pelegrin Jaime
2020-06-17
*/

#define FastAudioFIFO_SIZE 2048 //MUST BE POWER OF 2 !!!
#define FastAudioFIFO_MASK (FastAudioFIFO_SIZE-1)

class FastAudioFIFO 
{
public:

	void init(void)
	{
		head_ = 0;
		tail_ = 0;
	}

	bool put(int16_t item)
	{
		//if (full())
		//	return false;

		//std::lock_guard<std::mutex> lock(mutex_);
		buf_[(head_++) & FastAudioFIFO_MASK] = item;

		return true;
	}

	bool get(int16_t* item)
	{
		//std::lock_guard<std::mutex> lock(mutex_);

		if (empty())
			return false;

		*item = buf_[(tail_++) & FastAudioFIFO_MASK];

		return true;
	}

	void reset(void)
	{
		//std::lock_guard<std::mutex> lock(mutex_);
		head_ = tail_;
	}

	bool empty(void) const
	{
		//if head and tail are equal, we are empty
		return head_ == tail_;
	}

	bool full(void) const
	{
		//If tail is ahead the head by 1, we are full
		return ((head_ + 1) & FastAudioFIFO_MASK) == (tail_ & FastAudioFIFO_MASK);
	}

	size_t len(void) const
	{
		return head_ - tail_;
	}

	size_t available(void) const
	{
		return  FastAudioFIFO_SIZE - (head_ - tail_);
	}

private:
	//std::mutex mutex_;
	int16_t buf_[FastAudioFIFO_SIZE];
	size_t head_ = 0;
	size_t tail_ = 0;
};
