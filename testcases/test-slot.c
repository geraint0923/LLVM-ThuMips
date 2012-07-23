
int main()
{
		short a;
		short b;
		short c;
		a = 11;
		b = 22;
label:
		c += b - a;
		if(c)
				goto label;
		++c;
		return 0;
}
