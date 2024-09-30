def print_analytics(analytics,steps):
    print("Analytics: ")
    total =0
    for k,v in analytics["TIMES"].items():
        print(f"{k}: {v}")
        total += v
    print("Total: ",total)
    print("Average_step_cnt: ",analytics["SUM_STEP_CNT"]/steps)
    print("Filled_cnt: ",analytics["FILLED_CNT"])
    print("Collided_cnt: ",analytics["COLLIDED_CNT"])