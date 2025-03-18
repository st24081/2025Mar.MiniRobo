struct Flag
{
  bool suspension;

  bool operator()()
  {
    return suspension;
  }

  void operator()(bool flag)
  {
    suspension = flag;
  }
};