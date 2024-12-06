def fibonnacci(n):
    tempa = 0
    tempb = 1
    for i in range(n):
        tempa = tempb
        tempb += i
        print(tempa+tempb)
fibonnacci(10)