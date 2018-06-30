#!/usr/bin/Rscript
#
# CopterLogAnalysis.R - Statistical multicopter energy consumption
# analysis based on Ardupilot dataflash log files.
# Copyright (C) 2017  Thomas Dietrich
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# Clear workspace
remove(list = ls())
# Close all devices still open from previous executions
graphics.off()

#####################################################################
# Logfiles to analyze ###############################################

#####################################################################
# Settings ##########################################################

logdata_folder <- "../results/"
tikzLocation = "./tikz/"

#overall OMNeT++ simulation time in seconds
simtimeSec <- 12 * 3600

# http://www.stat.columbia.edu/~tzheng/files/Rcolor.pdf
#graphCol1 <- "steelblue"
#graphCol2 <- "goldenrod"
#graphCol2_dark <- "goldenrod4"
#graphCol3 <- "gray70"

#graphCol1 <- "#353594"
graphCol1 <- "#4682B4" #steelblue
graphCol2 <- "#B8850A" #mygold
graphCol2_dark <- "goldenrod4"
graphCol3 <- "#B3B3B3" #mygray
graphCol3_dark <- "#737373"
graphCol3_darkdark <- "#434343"
graphCol4 <- "#780116" #myred

#####################################################################
# Install packages, load libraries ##################################

list.of.packages <- c("Hmisc", "geosphere", "ggplot2", "cowplot", "tikzDevice", "TTR", "xts", "forecast",
                      "data.table", "xtable", "stringr")
new.packages <- list.of.packages[!(list.of.packages %in% installed.packages()[,"Package"])]
if(length(new.packages)) install.packages(new.packages)
sapply(list.of.packages, require, character.only = TRUE)
lapply(list.of.packages, packageVersion)
remove(list = c("list.of.packages", "new.packages"))

# Increase max number of warnings
options(nwarnings=200) 

### Update packages (from time to time!)
#update.packages(checkBuilt=TRUE, ask=FALSE)

# plotlyUsername <- "user"
# plotlyApiKey <- "key"
#source("plotlyCredentials.R")

#Sys.setenv("plotly_username" = plotlyUsername)
#Sys.setenv("plotly_api_key" = plotlyApiKey)
#remove(plotlyUsername, plotlyApiKey)


theme_custom <- function () {
  theme_light() %+replace% 
    theme(
      panel.background  = element_blank(),
      axis.title = element_text(size = rel(0.8)),
      axis.ticks=element_blank(),
      legend.text = element_text(size = rel(0.8)),
      legend.title = element_text(size = rel(0.8)),
      panel.border=element_blank()
    )
}
theme_set(theme_custom())

options(tikzDefaultEngine = "xetex")

#####################################################################

df.all <- data.frame()

for (filename in list.files(logdata_folder,"*.sca")) {
  logdata.file <- data.frame()
  cat(filename)
  cat(": ")
  filename.replM <- NA
  filename.quant <- NA
  filename.numUAVs <- NA
  
  filename.basename <- str_replace(filename, "(.*?)-.*", "\\1")
  
  filename.replM   <- ifelse(grepl("replM=", filename), str_replace(filename, ".*-replM=(\\d).*", "\\1"), NA)
  filename.quant   <- ifelse(grepl("quant=", filename), str_replace(filename, ".*-quant=(\\d).*", "\\1"), NA)
  filename.numUAVs <- ifelse(grepl("numUAVs=", filename), str_replace(filename, ".*-numUAVs=(\\d).*", "\\1"), NA)
  
  filename.repeat <- str_replace(filename, ".*-#(\\d+).*", "\\1")
  
  cat("Load File ... ")
  lines <- readLines(paste(logdata_folder, filename, sep=""))
  #head(lines)
  lines.scalar <- grep("scalar OsgEarthNet", lines, value=TRUE)
  #head(lines.scalar)
  
  cat("Parse Scalars ... ")
  for (line in lines.scalar) {
    node.type <- str_replace(line, "scalar OsgEarthNet\\.(.*?)\\[\\d+\\].*", "\\1")
    index <- str_replace(line, "scalar OsgEarthNet.*?\\[(\\d+)\\].*", "\\1")
    metric <- str_replace(line, "scalar OsgEarthNet.*?\\[\\d+\\] (\\w*).*", "\\1")
    value <-str_replace(line, "scalar OsgEarthNet.*?\\[\\d+\\] \\w* (.*)", "\\1")
    #print(paste(index, metric, value, sep = " --- "))
    
    logdata.line <- data.frame(
      basename =         as.factor(filename.basename),
      replM =            as.integer(filename.replM),
      quant =            as.integer(filename.quant),
      numUAVs =          as.integer(filename.numUAVs),
      simRun =           as.integer(filename.repeat),
      nodeType =         as.factor(node.type),
      index =            as.integer(index),
      metric =           as.factor(metric),
      value =            as.numeric(value)
    )
    logdata.file <- rbind(logdata.file, logdata.line)
  }
  cat("\n")
  df.all <- rbind(df.all, logdata.file)
}

# Noteworthy general data
head(df.all)
replMs <- unique(df.all$replM)
quant <- unique(df.all$quant)
numUAVs <- unique(df.all$numUAVs)

# Split types
df.all.uav <- subset(df.all, nodeType == 'uav')
df.all.cs <- subset(df.all, nodeType == 'cs')

#####################################################################

uavs_count <- max(df.all.uav$index)
sim_runs <- unique(df.all.uav$simRun)
metrics <- levels(factor(df.all.uav$metric))


cat("Remove run+UAVs with 'fail' metric ... ")

uavs_all <- subset(df.all.uav, metric == 'utilizationFail')
uavs_all_failed <- subset(df.all.uav, metric == 'utilizationFail' & value != 0)

if (nrow(uavs_all_failed) > 0) {
  warning(paste(nrow(uavs_all_failed), "of", nrow(uavs_all), "UAVs over all simulation runs ended in 'fail' state", sep=" "))
}

if (nrow(uavs_all_failed) > 0) {
  for(i in 1:nrow(uavs_all_failed)) {
    row <- uavs_all_failed[i,]
    #print(row)
    df.all.uav <- df.all.uav[!(df.all.uav$replM==row$replM & df.all.uav$simRun==row$simRun & df.all.uav$index==row$index),]
  }
}
remove(uavs_all, uavs_all_failed)



cat("Proofchecking Simtime ... ")
tolerance = 0.1
for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (index in 0:uavs_count) {
      df_subset <- subset(df.all.uav, replM == replMethod & simRun == sim_run & index == index
                          & metric %in% c('utilizationSecMaintenance', 'utilizationSecMission', 'utilizationSecCharge', 'utilizationSecIdle')
      )
      if (nrow(df_subset) > 0) {
        #print(sum(df_subset$value))
        if (abs(simtimeSec - sum(df_subset$value)) > tolerance) warning("Simtime missmatch!")
      }
    }
  }
}

#####################################################################

cat("Combining SimRuns ... ")
df.comb <- data.frame()

for (replMethod in replMs) {
  for (index in 0:uavs_count) {
    for (metric in metrics) {
      df_subset <- subset(df.all.uav, replM == replMethod & index == index & metric == metric)
      metric_value.mean <- sum(df_subset$value)
      metric_value.stddev <- sqrt(var(df_subset$value))
      #print(paste(replMethod, uav, metric, metric_value.mean, metric_value.stddev, sep = " -- "))
      df <- data.frame(
        basename =    levels(df_subset$basename),
        replM =       as.integer(replMethod),
        index =       as.integer(index),
        metric =      as.factor(metric),
        value =       as.numeric(metric_value.mean),
        valueStddev = as.numeric(metric_value.stddev)
      )
      df.comb <- rbind(df.comb, df)
    }    
  }
}
#remove(replMethod, uav, metric, df_subset, metric_value.mean, metric_value.stddev, df)

#TEMPORARY
df.comb <- df.all.uav


cat("Summarizing Metrics per UAV... ")
df.red <- data.frame()

for (replMethod in replMs) {
  for (sim_run in sim_runs) {
    for (uav_index in 0:uavs_count) {
      df_subset <- subset(df.comb, replM == replMethod & simRun == sim_run & index == uav_index)
      df <- data.frame(
        basename =    levels(df_subset$basename),
        replM =       as.integer(replMethod),
        simRun =      as.integer(sim_run),
        index =       as.integer(uav_index),
        #
        secMission =                  as.numeric(subset(df_subset, metric == 'utilizationSecMission')$value),
        secMaintenance =              as.numeric(subset(df_subset, metric == 'utilizationSecMaintenance')$value),
        secCharge =                   as.numeric(subset(df_subset, metric == 'utilizationSecCharge')$value),
        secIdle =                     as.numeric(subset(df_subset, metric == 'utilizationSecIdle')$value),
        #
        energyMission =               as.numeric(subset(df_subset, metric == 'utilizationEnergyMission')$value),
        energyMaintenance =           as.numeric(subset(df_subset, metric == 'utilizationEnergyMaintenance')$value),
        energyCharge =                as.numeric(subset(df_subset, metric == 'utilizationEnergyCharge')$value),
        #
        energyOverdrawMission =       as.numeric(subset(df_subset, metric == 'utilizationEnergyOverdrawMission')$value),
        energyOverdrawMaintenance =   as.numeric(subset(df_subset, metric == 'utilizationEnergyOverdrawMaintenance')$value),
        #
        countMissions =               as.numeric(subset(df_subset, metric == 'utilizationCountMissions')$value),
        countManeuversMission =       as.numeric(subset(df_subset, metric == 'utilizationCountManeuversMission')$value),
        countManeuversMaintenance =   as.numeric(subset(df_subset, metric == 'utilizationCountManeuversMaintenance')$value),
        countChargeState =            as.numeric(subset(df_subset, metric == 'utilizationCountChargeState')$value),
        countOverdrawnAfterMission =  as.numeric(subset(df_subset, metric == 'utilizationCountOverdrawnAfterMission')$value),
        countIdleState =              as.numeric(subset(df_subset, metric == 'utilizationCountIdleState')$value)
      )
      df.red <- rbind(df.red, df)
    }
  }
}

#####################################################################
# Diagrams ##########################################################
cat("Drawing Diagrams ... ")

starting_cs <- as.factor(df.red$index %% 3)

# Test correlation with starting position (CS 0,1, or 2)
ggplot(df.red, aes(starting_cs, secIdle)) + geom_boxplot() + geom_jitter(width = 0.2)


# Diagrams
ggplot(df.red) +
  geom_bin2d(aes(x = 100 / (secMission + secMaintenance) * secMission, y = 100 / (secMission + secMaintenance) * secMaintenance)) +
  labs(x="Time in Mission", y="Time in Maintenance")

ggplot(df.red) +
  geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) +
  labs(x="Energy in Mission", y="Energy in Maintenance")


ggplot(subset(df.red, replM==0)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))
ggplot(subset(df.red, replM==1)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))
ggplot(subset(df.red, replM==2)) + geom_bin2d(aes(x = energyMission, y = energyMaintenance))

ggplot(subset(df.red, replM==0)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)
ggplot(subset(df.red, replM==1)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)
ggplot(subset(df.red, replM==2)) + geom_bin2d(aes(x = 100 / (energyMission + energyMaintenance) * energyMission, y = 100 / (energyMission + energyMaintenance) * energyMaintenance)) + ylim(0, 100) + xlim(0, 100)
